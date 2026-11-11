function ap = lindiCopterAutoCreate( copter, varargin )
% lindiCopterAutoCreate automatic computation of LindiCopter autopilot
% parameters
% 
% Syntax:
%   ap = lindiCopterAutoCreate( copter )
%   ap = lindiCopterAutoCreate( copter, Name, Value )
% 
% Inputs:
%   copter          multicopter parameters struct, see copterLoadParams
%   Name            name of Name-Value Arguments:
%                       - 'AgilityAtti': define agility of attitude
%                           controller with values between -1 ... 1
%                           (-1: low, 0: medium, 1: high), default: 0
%                       - 'AgilityPos': define agility of position
%                           controller with values between -1 ... 1
%                           (-1: low, 0: medium, 1: high), default: 0
%                       - 'FilterStrength': define sensor low-pass filter
%                           strength with values between -1 ... 1 (-1:
%                           weak, 0: medium, 1: strong), default: 0
%                       - 'CntrlEffectScaling': define scaling of control
%                           effectiveness with values between 0.75 ... 1
%                           (0.75: reduce to 75%, 1: no scaling),
%                           default: 1
%   Value           value of Name-Value Arguments (see input Name)
% 
% Outputs:
%   ap              LindiCopter autopilot parameters struct as defined by
%                   this function

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
%
% ═══════════════════════════════════════════════════════════════════════
% 이 함수의 원리 (우리 연구팀 주석)
%   입력: copter(기체 사양: 질량·관성·모터·프로펠러) + 다이얼(aA·aP 등)
%   출력: ap = fm_loiter (제어기 파라미터: 극점·게인·제어할당·필터)
%   ※ 만드는 건 "제어기 설정"이지 "기체 사양"이 아님. 기체 사양은 입력.
%
%   4단계 원리:
%    ① 공격성 변환:  다이얼 aA/aP(-1~1) → aggressiveness(0.5~1)  [69행]
%    ② 물리 한계:    모터가 낼 수 있는 최대 추력·각가속도 계산    [100~244행]
%    ③ 대역폭 결정:  ①목표와 ②물리한계·시간척도분리 중 보수적 값 [246~264행]
%    ④ 극점배치:     원하는 극점 → 실제 게인 역산(k.lean 등)      [266~292행]
%   → 화물로 관성↑ 시 ②에서 대역폭 천장↓ → ④ 게인 자동조정(INDI 궁합).
% ═══════════════════════════════════════════════════════════════════════

% initialize default tuning parameters
% 다이얼 기본값(0=medium). aA·aP는 우리가 BO로 탐색하는 값.
agility_atti                = 0;   % 자세 공격성 (AgilityAtti, aA)
agility_pos                 = 0;   % 위치 공격성 (AgilityPos, aP)
filter_strength             = 0;   % 센서 저역통과 필터 강도
cntrl_effect_scaling_factor = 1;   % 제어효과 스케일(0.75~1)

% parse name-value arguments
for i = 1:length(varargin)
    if ischar(varargin{i})
        if isequal(varargin{i},'AgilityAtti')
            agility_atti(:) = varargin{i+1};
        elseif isequal(varargin{i},'AgilityPos')
            agility_pos(:) = varargin{i+1};
        elseif isequal(varargin{i},'FilterStrength')
            filter_strength(:) = varargin{i+1};
        elseif isequal(varargin{i},'CntrlEffectScaling')
            cntrl_effect_scaling_factor(:) = varargin{i+1};
        end
    end
end

% assure tuning parameter limits
agility_atti(:) = max( min(agility_atti,1), -1 );
filter_strength(:) = max( min(filter_strength,1), -1 );
ce_scaling_max = 1;
ce_scaling_min = 0.75;
cntrl_effect_scaling_factor(:) = max( ...
    min(cntrl_effect_scaling_factor,ce_scaling_max), ce_scaling_min );

% aggressiveness ( aggr_min ... 1 )
% ① 공격성 변환: 다이얼 agility(-1~1) → aggressiveness(0.5~1).
%   agility=1(공격)→aggr=1, agility=-1(보수)→aggr=0.5, 기본0→0.75.
%   이 aggr가 이후 대역폭·극점을 결정 → aA가 곧 "제어 빠르기 다이얼".
aggr_min = 0.5;
aggr_atti = 1 - 0.5*(1-aggr_min)*(1-agility_atti);   % 자세 공격성
aggr_pos = 1 - 0.5*(1-aggr_min)*(1-agility_pos);     % 위치 공격성

% filter cutoff factor ( 1/4 ... 4 )
% 센서 필터 차단주파수 배수. 강하면(filter_strength↑) 대역 좁힘.
filter_factor = 4^(-filter_strength);

% time scale separation factor (2-5)
% ③의 핵심: 시간척도 분리계수. 자세루프가 위치루프보다 몇 배 빠른지.
%   보수적(aggr 낮음)일수록 분리 크게(5) → 더 안정. 캐스케이드 제어 원칙.
sep_factor_min = 2;
sep_factor_max = 5;
sep_factor_atti = sep_factor_min + (1-aggr_atti)/(1-aggr_min)*(sep_factor_max-sep_factor_min);
sep_factor_pos = sep_factor_min + (1-aggr_pos)/(1-aggr_min)*(sep_factor_max-sep_factor_min);



is_flipped_allowed = 1;

g = 9.81;

% ★ 제어할당(CA) 가중치 W_v — passive FTC의 심장.
%   원하는 [롤,피치,요,추력]을 모터로 분배할 때 각 축의 우선순위.
%   요(3번)=0.01로 사실상 포기 → 로터1개 고장 시 기체를 스핀시켜(자이로
%   안정) 롤·피치·추력만 살림. 이게 "고장 감지 불필요"의 실제 구현 지점.
ap.ca = loadParams( 'caWls_params_default' );
ap.ca.W_v(1,1) = 30;    % 롤 우선
ap.ca.W_v(2,2) = 30;    % 피치 우선
ap.ca.W_v(3,3) = 0.01;  % 요 ← 거의 0 = 포기(스핀 허용)
ap.ca.W_v(4,4) = 300;   % 추력 최우선(고도 유지)

% maximum number of waypoints
ap.traj.wpmax = 4;
% boolean if last and first waypoint should be connected (true) or not
ap.traj.cycle = true;
% degree of spline polynomials
ap.traj.degree = 5;

num_motors = size(copter.config.propPos_c,2);

[k,d] = propMapFitGetFactors(copter.prop.map_fit);
omega_max = motorStaticSpeed(copter.motor.KT,copter.motor.R,copter.bat.V,d,ap.ca.u_max);
omega_min = motorStaticSpeed(copter.motor.KT,copter.motor.R,copter.bat.V,d,ap.ca.u_min);


% ② 물리 한계 계산 시작: 추력 ∝ k·ω².
%   thrust_hover = 무게(mg) → 호버에 필요한 총추력.
%   omega_hover = 호버 시 모터 회전수(→ 우리 무게추정이 쓰는 Σω²의 기준).
thrust_max = sum( k*omega_max.^2 );        % 모터 최대 총추력
thrust_min = sum( k*omega_min.^2 );        % 모터 최소 총추력
thrust_hover = copter.body.m * g;          % 호버 추력 = 무게

omega_hover = sqrt( thrust_hover / num_motors / k );   % 호버 회전수
torque_hover = d * omega_hover.^2;         % 호버 토크

% u_hover = 호버 유지에 드는 정규화 모터명령(0~1).
%   0.95 넘으면 = 추력 여유 거의 없음 → 호버 불가 에러.
%   (우리 화물 실험: 화물↑ 시 u_hover↑, 고장 후 남은 3모터로는 포화 위험.)
u_hover = (torque_hover/copter.prop.I / (copter.motor.KT/(copter.motor.R*copter.prop.I)) + copter.motor.KT*omega_hover) / copter.bat.V;
if u_hover > 0.95
    error('Not enough thrust to hover.')
end
u_hover_2 = u_hover^2;
u_min_2 = ap.ca.u_min.^2;
u_max_2 = ap.ca.u_max.^2;
ap.thr.min = sqrt(u_hover_2-0.6*(u_hover_2-u_min_2));
ap.thr.max = sqrt(u_hover_2+0.75*(u_max_2-u_hover_2));

delta_thrust_max = min( thrust_max - thrust_hover, thrust_hover - thrust_min );

omega_delta_max = sqrt( ( thrust_hover + delta_thrust_max ) / k );
omega_delta_min = sqrt( ( thrust_hover - delta_thrust_max ) / k );

torque_delta_max = d * omega_delta_max^2;
torque_delta_min = d * omega_delta_min^2;

acc_up_max = thrust_max / copter.body.m;
acc_down_max = thrust_min / copter.body.m;

ap.psc.rm.accumax = aggr_pos * ( acc_up_max - g );
ap.psc.rm.accdmax = aggr_pos * ( g - acc_down_max );

prop_name_cell = strsplit(copter.prop.name,'x');
prop_diameter = str2num(prop_name_cell{1}) * 2.54/100;
A = num_motors * pi/4 * prop_diameter^2;
[ ~, ~, v_i0 ] = inducedVelocityWithUnitsAngleOfAttack( 0, -pi/2, thrust_hover, 1.225, A );


% temp: to do
ap.psc.rm.veldmax = aggr_pos * v_i0;


omega = 0;
V = 0;
dt = 0.01;
while true
    u = aggr_pos;
    torque = propMapFitGetZ(copter.prop.map_fit,omega*60/(2*pi),V,'torque');
    thrust = propMapFitGetZ(copter.prop.map_fit,omega*60/(2*pi),V,'thrust');
    dot_omega = copter.motor.KT/copter.motor.R/copter.prop.I*(copter.bat.V*u-copter.motor.KT*omega)-torque/copter.prop.I;
    acc = 1/copter.body.m * ( num_motors*thrust - 0.5*1.225*copter.aero.S*copter.aero.C_Dmax*V^2 );
    omega = omega + dot_omega*dt;
    V = V + acc*dt;
    if abs(dot_omega) < 0.1 && abs(acc) < 0.01
        break;
    end
end

ap.psc.rm.velumax = aggr_pos * V;
ap.psc.rm.velxymax = aggr_pos * V;

lean_max = acos( g / acc_up_max );
if isreal(lean_max)
    acc_xy_max = acc_up_max * sin( lean_max );
else
    error('Not enough thrust to hover.')
end
if is_flipped_allowed
    ap.atc.rm.leanmax = pi;
else
    ap.atc.rm.leanmax = lean_max;
end
ap.atc.rm.leandamp = 1;
ap.atc.rm.yawratemax = 2*pi;

ap.psc.rm.accxymax = aggr_pos * acc_xy_max;
ap.psc.rm.veltc = 1.25/aggr_pos * ap.psc.rm.velxymax / ap.psc.rm.accxymax;



M = zeros(3,num_motors);
for i=1:num_motors
    M(:,i) = evalin('caller',['copter.config.M_b_prop',num2str(i)]) * [1;0;0];
end

% propeller control effectiveness parameters
ap.cep.k 	= k;
ap.cep.d	= d;
ap.cep.x	= copter.config.propPos_c(1,:) - copter.config.CoG_Pos_c(1);
ap.cep.y	= copter.config.propPos_c(2,:) - copter.config.CoG_Pos_c(2);
ap.cep.z	= copter.config.propPos_c(3,:) - copter.config.CoG_Pos_c(3);
ap.cep.a	= copter.config.propDir(:)';
ap.cep.nx	= M(1,:);
ap.cep.ny	= M(2,:);
ap.cep.ip	= copter.prop.I;
ap.cep.kt	= copter.motor.KT;
ap.cep.vb	= copter.bat.V;
ap.cep.ri	= copter.motor.R;

% body control effectiveness parameters
% ★ 제어효과(body): 기체 질량·관성텐서를 제어기에 직접 넣는 곳.
%   INDI가 "이만큼 모멘트 주면 이만큼 각가속" 계산할 때 이 관성을 씀.
%   ★ 우리 mkplant가 copter.body.I를 바꾸면 여기로 흘러들어 제어기가
%     화물 관성을 인지. 편심(ixz≠0) 실험에서 이 값이 발산/회복을 갈랐음.
ap.ceb.m    = cntrl_effect_scaling_factor * copter.body.m;
ap.ceb.ixx  = cntrl_effect_scaling_factor * copter.body.I(1,1);   % 롤 관성
ap.ceb.iyy  = cntrl_effect_scaling_factor * copter.body.I(2,2);   % 피치 관성
ap.ceb.izz  = cntrl_effect_scaling_factor * copter.body.I(3,3);   % 요 관성
ap.ceb.ixy  = cntrl_effect_scaling_factor * -copter.body.I(1,2);
ap.ceb.ixz  = cntrl_effect_scaling_factor * -copter.body.I(1,3);  % 비대각(편심)
ap.ceb.iyz  = cntrl_effect_scaling_factor * -copter.body.I(2,3);



L = cross( copter.config.propPos_c, M );

thrust_vector = zeros( num_motors, 1 );
torque_vector = zeros( num_motors, 1 );


thrust_vector(L(1,:)'>0) = ( thrust_hover + delta_thrust_max ) / num_motors;
thrust_vector(L(1,:)'<0) = ( thrust_hover - delta_thrust_max ) / num_motors;
roll_moment_max = L(1,:) * thrust_vector;

% ② 최대 롤 각가속도 = 최대 롤모멘트 / 롤관성.
%   관성↑(화물)이면 같은 모멘트로도 각가속↓ → ③에서 대역폭 천장 낮아짐.
acc_roll_max = roll_moment_max / copter.body.I(1,1);

thrust_vector(L(2,:)'>0) = ( thrust_hover + delta_thrust_max ) / num_motors;
thrust_vector(L(2,:)'<0) = ( thrust_hover - delta_thrust_max ) / num_motors;
pitch_moment_max = L(2,:) * thrust_vector;

acc_pitch_max = pitch_moment_max / copter.body.I(2,2);

acc_roll_pitch_max = min( acc_roll_max, acc_pitch_max );


torque_vector(ap.cep.a>0) = torque_delta_max/num_motors;
torque_vector(ap.cep.a<0) = -torque_delta_min/num_motors;
yaw_moment_max = sum( torque_vector );

acc_yaw_max1 = yaw_moment_max / copter.body.I(3,3);
Delta_u_yaw = 0.1;
acc_yaw_max2 = Delta_u_yaw * copter.motor.KT/copter.motor.R/copter.prop.I*copter.bat.V * num_motors * copter.prop.I/copter.body.I(3,3);
acc_yaw_max = mean([acc_yaw_max1,acc_yaw_max2]);

leanfreq_force = 2*pi*sqrt( 0.25*acc_roll_pitch_max / lean_max );
yawratetc_force = ap.atc.rm.yawratemax / acc_yaw_max;

ap.mtc = copter.motor.R*copter.prop.I/copter.motor.KT^2;

ap.ts = 0.0025;

% sensor filter
ap.sflt.D = 1;
ap.sflt.omega = filter_factor*2/ap.mtc;

ap.mtc = 1.2*ap.mtc;

T_h_1 = ap.mtc + 2/ap.sflt.omega;

leanfreq_sep = 1/sep_factor_atti*2/( T_h_1 );
yawratetc_sep = sep_factor_atti*( T_h_1 );
% ③ 대역폭 결정: 자세 고유진동수 leanfreq.
%   leanfreq_force(물리 한계) vs leanfreq_sep(시간척도 분리) 중 min(보수적).
%   → 모터가 못 따라가는 빠른 제어도, 위치루프와 안 분리된 제어도 배제.
%   ★ aA를 낮추면 이 leanfreq가 낮아짐 = 하한 0.3 결함이 여기서 작동.
ap.atc.rm.leanfreq = min( leanfreq_force, leanfreq_sep );
ap.atc.rm.yawratetc = max( yawratetc_force, yawratetc_sep );

% lean angle controller
% ④ 극점배치: 원하는 극점(-leanfreq 3중근)을 만드는 게인을 역산.
%   ndiFeedbackGainPlace가 3차 오차동역학의 극점을 -leanfreq에 놓는 게인 계산.
%   ★ 이 k(1)~k(3)가 곧 Minnie가 손으로 정했던 k.lean=200 등에 해당.
%     = 우리가 aA만 돌리면 이 게인 3개가 자동으로 일관되게 바뀌는 지점.
% max_roll_pitch_atti_error = 2;
% k = maxError2FeedbackGain( max_roll_pitch_atti_error, acc_roll_pitch_max, T_h, aggr );
k = ndiFeedbackGainPlace(-ap.atc.rm.leanfreq*[1,1,1],T_h_1);
ap.atc.k.lean = k(1);        % 자세각 오차 게인
ap.atc.k.leanrate = k(2);    % 자세각속도 게인
ap.atc.k.leanacc = k(3);     % 자세각가속도 게인

% yaw controller
% max_yaw_atti_error = pi;
% k = maxError2FeedbackGain( max_yaw_atti_error, acc_yaw_max, T_h, aggr );
k = ndiFeedbackGainPlace(-1/ap.atc.rm.yawratetc*[1,1,1],T_h_1);
ap.atc.k.yaw = k(1);
ap.atc.k.yawrate = k(2);
ap.atc.k.yawacc = k(3);

% position feedback controller
% 위치 제어 극점배치. 자세루프 지연(2/leanfreq)을 더해 T_h_pos 산정
%   → 위치루프는 자세루프보다 느리게(캐스케이드). aP가 sep_factor_pos로 조정.
T_h_pos = T_h_1 + 2/ap.atc.rm.leanfreq;
% p = -0.5*[1+1i,1-1i,4] * aggr / T_h;
% p = -0.5*[1,1,1] * aggr / T_h;
p = -1.3/sep_factor_pos * [1.7,1+0.65i,1-0.65i] / T_h_pos;
% p = -sep_factor*aggr*[ 1/T_h, 0.7/T_h*(1+0.65i), 0.7/T_h*(1-0.65i) ];
% p = -aggr/sep_factor*[ 1, 1, 1 ]*2/T_h;
k = ndiFeedbackGainPlace(p,T_h_pos);
ap.psc.k.pos = k(1);
ap.psc.k.vel = k(2);
ap.psc.k.acc = k(3);

end
