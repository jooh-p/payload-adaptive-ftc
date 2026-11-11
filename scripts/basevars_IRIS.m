function V = basevars_IRIS(cdry, m_dry, meta, PM, POFF, T, TEND, withfail)
%% basevars_IRIS — 시뮬 변수를 구조체로 모은다
%
% 변수를 base 작업공간이 아니라 구조체로 모아 SimulationInput.setVariable 로
% 넘긴다. 이렇게 하면 for 와 parfor 어느 쪽에서도 같게 동작한다
% (parfor 워커는 주 세션의 base 작업공간을 보지 못한다). 실행은 simseg_IRIS.
%
% 입력
%   cdry     무화물 기체 구조체
%   m_dry    무화물 질량 [kg]
%   meta     abc_IRIS.mat 의 ABC.meta
%   PM       화물 질량 [kg]
%   POFF     장착 깊이 [m]
%   T        시나리오 함수의 두 번째 출력 (scn_delivery3_IRIS)
%   TEND     시뮬 종료 시각 [s]
%   withfail true 면 모터 2번을 T.TFAIL 에 정지시킨다
%
% 출력
%   V        setVariable 로 넘길 변수 구조체
%            제어기(fm_loiter) · 선제 추력(ffpar) · 감지 정지 스위치(dseg_stop_en)는
%            구간마다 바뀌므로 applyctrl_IRIS 로 얹는다.

V.copter = mkplant_payload_IRIS(cdry, PM, POFF);        % 화물 상자 관성 포함
V.envir  = envirLoadParams('params_envir','envir',0);   % 지면 고도 0
yaw0 = 0;  if isfield(T,'YAW0'), yaw0 = T.YAW0; end      % 초기 요는 시나리오가 정한다
V.IC     = struct('omega_Kb',[0;0;0], 'q_bg',euler2Quat([0;0;yaw0]), ...
                  'V_Kb',[0;0;0], 's_Kg',[0;0;0], 'omega_mot',[1;1;1;1]*480);
V.grnd   = groundLoadParams('params_ground_default');
V.pos_ref = struct('lat',37.6117, 'lon',-122.37822, 'alt',T.ALT0);  % 지상에서 시작
V.fg      = struct('remoteURL','127.0.0.1', 'remotePort',5502);
V.westpar = [m_dry; meta.sumsq_dry; T.EST(1); T.EST(2)];            % 질량 추정 — 무화물 질량 · 기준 회전수 제곱합 · 추정 창
V.simin   = scn_delivery3_IRIS(TEND);                               % 배송 시나리오

for j = 1:4                                                          % 고장 시각 (고장 없는 모터는 1000 s)
    if withfail && j==2, ft = T.TFAIL; else, ft = 1000; end
    V.(sprintf('failure_time_mot_%d', j)) = ft;
end
end
