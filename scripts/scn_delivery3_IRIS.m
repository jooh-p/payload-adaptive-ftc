function [simin, T] = scn_delivery3_IRIS(TEND, CR_AMP, YR_AMP, TURN_DUR, NOSTOP)
%% scn_delivery3_IRIS — 배송 시나리오 3판 (꼬리표 dlv3 · 1판 우회배송 dlv1 · 2판 배송 dlv2) (순항 5 m/s · 출발/정지 램프)
%
% ┌─ 1판(scn_delivery_IRIS)과 다른 점 둘 ─────────────────────────────
% │ ① 순항 속도 3.23 → 5.0 m/s   근거 PX4 MPC_XY_CRUISE 기본값 5.0 m/s
% │ ② 출발·정지를 계단이 아니라 램프로  근거 PX4 MPC_ACC_HOR 기본값 3 m/s²
% │      램프 길이 = 5.0 / 3.0 = 1.67 s, 0.1 s 간격 점으로 채움
% │
% │ ★ 스틱 표는 점 사이를 잇지 않고 앞 값을 유지한다(interpHold, 2026-09-08
% │   확인). 1판의 "1초 램프"는 실제로 뒤쪽 시각의 계단이었다. 램프는 점을
% │   촘촘히 찍어야만 램프가 된다.
% │
% │ 안 바꾼 것: 이륙·상승 스로틀(0.05 s 계단, 도달 고도 계수 18.27 이 이것으로
% │ 보정됨) · 무게추정 창 · 고장 35 s · 정지 명령 끝 56 s · 호버 끝 76 s.
% │ 순항 거리는 약 27 s × 5 = 135 m 로 늘어난다(부수 결과).
% └─────────────────────────────────────────────────────────────────────
%
% ┌─ 선회 추가 (2026-10-03 연구자 결정 · 옛 판은 옆 이동) ─────────────
% │ 기체 전면이 **북쪽**(요 0°)인 채 시작하고, 50 m 상승·정착이 끝나는 26.33 s 에
% │ 요 스틱으로 시계 방향 90° · 45 °/s(약 2 s) 선회해 **동쪽**을 향한 뒤
% │ **전면 방향(동쪽)으로 직진**한다(채널 1). 옛 판은 요 0°(전면 북쪽)인 채
% │ 롤 스틱으로 옆 이동(동쪽)이었다. 연구자 지시(2026-10-03): 북쪽에서 시작해
% │ 동쪽으로 돌려도 좋다 — 초기 조건·목적지 방향은 옛 판과 같다.
% └─────────────────────────────────────────────────────────────────────
%
% ┌─ 시간표 ────────────────────────────────────────────────────────────
% │  0     ~  1.00   땅에 앉음 (전면 북쪽)
% │  1.05  ~  3.24   이륙 스로틀 0.05 (계단)         → 2.00 m
% │  3.24  ~ 15.50   호버 · 무게추정 창 [13.24, 14.54]
% │ 15.55  ~ 21.80   상승 스로틀 0.4208 (계단)       → 50 m
% │ 21.80  ~ 26.33   정착
% │ 26.33  ~ 28.39   ★ 선회 90° (요 스틱 약 45 °/s, 전면 북쪽 → 동쪽, 시계 방향)
% │ 28.33  ~ 30.00   ★ 출발 램프 (0 → CR_AMP, 1.67 s)
% │ 30.00  ~ 54.33   순항 5 m/s 동쪽 (전면 방향)
% │ 35.00            ★ 2번 모터 고장
% │ 54.33  ~ 56.00   ★ 정지 램프 (CR_AMP → 0, 1.67 s)
% │ 56.00  ~ 76.00   호버
% │ 평가 창 — 초기 강하 [35, 45] · 후반 [45, 77] (cfg_current_IRIS)
% └─────────────────────────────────────────────────────────────────────
%
% 입력  TEND    시뮬 종료 [s] (기본 77)
%       CR_AMP  순항 스틱 진폭 (기본 = 5.0 m/s 실측 보정값). 보정용 인자
%       YR_AMP  선회 스틱 진폭 (기본 = 실제 요 각속도 최대 45.0 °/s 가 되는 값). 보정용 인자
%       TURN_DUR 선회 스틱 유지 시간 [s] (기본 = 정착 요가 90.0° 가 되는 값). 보정용 인자
%       NOSTOP  true 면 정지 램프를 없애고 순항 스틱을 끝까지 유지(진단용 · 기본 false)
% 출력  simin · T  (1판과 같은 구조) — T.YAW0 은 초기 요 [rad], basevars_IRIS 가 쓴다

if nargin < 1 || isempty(TEND), TEND = 77; end
if nargin < 5 || isempty(NOSTOP), NOSTOP = false; end

%% ── 설정 ──
T.ALT0     = 0.06;
T.K_ALT    = 18.27;
T.V_CRUISE = 5.0;                    % 목표 순항 속도 [m/s] (PX4 MPC_XY_CRUISE)
T.A_HOR    = 3.0;                    % 램프 가속도 [m/s²] (PX4 MPC_ACC_HOR)
T.CR_RAMP  = T.V_CRUISE / T.A_HOR;   % 램프 길이 1.67 s
T.DT_RAMP  = 0.1;                    % 램프 점 간격 [s]

% ★ 스틱 진폭 → 속도는 제곱 관계 (0.4208 → 3.234, 1.0 → 18.26). 5.0 m/s 는
%   계산으로 0.5233 이었으나 실측 4.61 m/s → 제곱 보정 0.5450 에서 5.000 m/s (test_scn3_IRIS).
if nargin < 2 || isempty(CR_AMP), CR_AMP = 0.5450; end   % ★ 실측 보정 2026-09-08: 0.5233 → 4.609 m/s, 0.5450 → 5.000 m/s
T.CR_AMP = CR_AMP;
% ★ 2026-10-03 보정 — 요 스틱 × 360 °/s 는 명령 기준이고 실제 요 각속도는 약 3% 더 나온다(0.125 → 최대 46.3 °/s).
%   실제 45.0 °/s 가 되도록 진폭을 낮추고, 줄어든 각을 유지 시간으로 채워 정착 요를 90.0° 에 맞춘다(`calibrate` 시험).
if nargin < 3 || isempty(YR_AMP), YR_AMP = 0.1214; end
if nargin < 4 || isempty(TURN_DUR), TURN_DUR = 2.06; end
T.YR_AMP = YR_AMP;

T.TK_T0  = 1.00;   T.TK_AMP = 0.05;    T.TK_DUR = 2.19;
T.HOV1   = 15.50;
T.EST    = [13.24, 14.54];
T.CL_AMP = 0.4208; T.CL_DUR = 6.25;
T.YAW0   = 0;                        % 초기 요 0° = 전면 북쪽 (basevars_IRIS 가 읽는다)
T.TURN_T0 = 28.00 - T.CR_RAMP;       % 선회 시작 26.33 (옛 출발 램프 시작 = 상승·정착 끝)
T.TURN_RATE = 45;                    % 선회 각속도 [°/s]
T.TURN_DUR  = TURN_DUR;              % 선회 스틱 유지 [s] — 명령상 90°/45 °/s = 2 s, 실제 응답 지연을 채워 2.06 s
T.TURN_T1 = T.TURN_T0 + T.TURN_DUR;  % 선회 스틱 끝 28.39
T.CR_T0  = T.TURN_T0 + 90 / T.TURN_RATE;   % 출발 램프 시작 28.33 (선회 스틱 끝과 0.06 s 겹치지만 속도 명령이 0 에서 시작)
T.CR_ON  = T.CR_T0 + T.CR_RAMP;      % 출발 램프 끝 = 순항 시작 30.00
T.TFAIL  = 35.00;
T.CR_OFF = 56.00;                    % 정지 램프 끝 = 호버 시작
T.CR_T1  = T.CR_OFF - T.CR_RAMP;     % 정지 램프 시작 54.33
T.HOV2   = 76.00;
T.TEND   = TEND;

T.ALT_TAKEOFF = T.TK_AMP*T.TK_DUR*T.K_ALT;
T.ALT_CRUISE  = T.ALT_TAKEOFF + T.CL_AMP*T.CL_DUR*T.K_ALT;

%% ── 스틱 시계열 ──
tk1 = T.TK_T0;  tk2 = tk1 + 0.05;  tk3 = T.TK_T0 + T.TK_DUR;  tk4 = tk3 + 0.05;
cl1 = T.HOV1;   cl2 = cl1 + 0.05;  cl3 = T.HOV1 + T.CL_DUR;   cl4 = cl3 + 0.05;

% 램프 점 — 앞 값 유지 방식이므로 점마다 값을 올린다
r1 = (T.CR_T0 : T.DT_RAMP : T.CR_ON).';   a1 = CR_AMP * (r1 - r1(1)) / T.CR_RAMP;
r2 = (T.CR_T1 : T.DT_RAMP : T.CR_OFF).';  a2 = CR_AMP * (1 - (r2 - r2(1)) / T.CR_RAMP);

% 채널별 (시각, 값) 사건 표 → 공통 시각으로 앞 값 유지. 채널 = 1 동쪽 · 2 북쪽(음수) · 3 요 · 4 스로틀
% ★ 수평 스틱은 기체가 아니라 **지면 좌표** 기준이다(채널 1 = 동쪽, 채널 2 = 북쪽(음수), 2026-10-03 확인).
%   선회 뒤 전면이 동쪽이므로 채널 1(+)이 곧 전진이다. 시계 방향 선회(북쪽 → 동쪽)는 요 스틱 **양수**
EV = cell(1,4);
EV{1} = [0 0; r1, a1; r2, a2; T.CR_OFF 0; T.HOV2 0];   % ★ 정지 램프 끝(56.0 s)에 0 점 — 없으면 마지막 점 0.022 가 76 s 까지 남는다(옛 판부터의 잔여)
if NOSTOP, EV{1} = [0 0; r1, a1; T.HOV2 CR_AMP]; end   % 진단용 — 정지 램프 없음
EV{2} = [0 0];
EV{3} = [0 0; T.TURN_T0 YR_AMP; T.TURN_T1 0];
EV{4} = [0 0; tk1 0; tk2 T.TK_AMP; tk3 T.TK_AMP; tk4 0; cl1 0; cl2 T.CL_AMP; cl3 T.CL_AMP; cl4 0];
t = unique([0; TEND; cell2mat(cellfun(@(e) e(:,1), EV.', 'UniformOutput', false))]);
t = t(t <= TEND);
n = numel(t);  v = zeros(n, 4);
for k = 1:4
    e = EV{k};
    for i = 1:n
        j = find(e(:,1) <= t(i) + 1e-9, 1, 'last');   % 앞 값 유지
        v(i,k) = e(j,2);
    end
end

if any(diff(t) <= 0)
    error('scn_delivery3_IRIS:시각순서', '시각이 단조증가가 아니다.');
end
if TEND < T.HOV2
    warning('scn_delivery3_IRIS:짧은종료', 'TEND(%g) 가 호버 끝(%g) 보다 이르다.', TEND, T.HOV2);
end

simin.time = t;
simin.signals.values = v;
simin.signals.dimensions = 4;
end
