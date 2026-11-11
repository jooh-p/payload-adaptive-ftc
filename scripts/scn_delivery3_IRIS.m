function [simin, T] = scn_delivery3_IRIS(TEND, CR_AMP, YR_AMP, TURN_DUR, NOSTOP)
%% scn_delivery3_IRIS — delivery mission scenario (90° turn · 5 m/s cruise · start/stop ramps)
%
% ┌─ Basis ─────────────────────────────────────────────────────────────
% │ Cruise speed 5.0 m/s        PX4 MPC_XY_CRUISE default
% │ Start/stop ramp 3 m/s²      PX4 MPC_ACC_HOR default — ramp length 5.0 / 3.0 = 1.67 s
% │
% │ ★ The stick table holds the previous value between points (interpHold).
% │   Ramps are made by placing points every 0.1 s.
% └─────────────────────────────────────────────────────────────────────
%
% ┌─ Turn ──────────────────────────────────────────────────────────────
% │ The vehicle starts facing north (yaw 0°). At 26.33 s, after climbing to 50 m and
% │ settling, it turns 90° clockwise at 45 °/s (about 2 s) with the yaw stick to face
% │ east, then flies straight ahead (east, channel 1).
% └─────────────────────────────────────────────────────────────────────
%
% ┌─ Timeline ──────────────────────────────────────────────────────────
% │  0     ~  1.00   on the ground (facing north)
% │  1.05  ~  3.24   take-off throttle 0.05 (step)    → 2.00 m
% │  3.24  ~ 15.50   hover · mass-estimation window [13.24, 14.54]
% │ 15.55  ~ 21.80   climb throttle 0.4208 (step)     → 50 m
% │ 21.80  ~ 26.33   settle
% │ 26.33  ~ 28.39   90° turn (yaw stick about 45 °/s, north → east, clockwise)
% │ 28.33  ~ 30.00   start ramp (0 → CR_AMP, 1.67 s)
% │ 30.00  ~ 54.33   cruise 5 m/s east (forward)
% │ 35.00            motor 2 failure
% │ 54.33  ~ 56.00   stop ramp (CR_AMP → 0, 1.67 s)
% │ 56.00  ~ 76.00   hover
% │ Evaluation windows — initial drop [35, 45] · late phase [45, 77] (cfg_current_IRIS)
% └─────────────────────────────────────────────────────────────────────
%
% Inputs  TEND     simulation end [s] (default 77)
%         CR_AMP   cruise stick amplitude (default = value giving an actual cruise speed of 5.0 m/s). Calibration argument
%         YR_AMP   turn stick amplitude (default = value giving an actual peak yaw rate of 45.0 °/s). Calibration argument
%         TURN_DUR turn stick hold time [s] (default = value giving a settled yaw of 90.0°). Calibration argument
%         NOSTOP   true = no stop ramp, cruise stick held to the end (default false)
% Outputs simin · T  — T.YAW0 is the initial yaw [rad], used by basevars_IRIS

if nargin < 1 || isempty(TEND), TEND = 77; end
if nargin < 5 || isempty(NOSTOP), NOSTOP = false; end

%% ── Settings ──
T.ALT0     = 0.06;
T.K_ALT    = 18.27;
T.V_CRUISE = 5.0;                    % target cruise speed [m/s] (PX4 MPC_XY_CRUISE)
T.A_HOR    = 3.0;                    % ramp acceleration [m/s²] (PX4 MPC_ACC_HOR)
T.CR_RAMP  = T.V_CRUISE / T.A_HOR;   % ramp length 1.67 s
T.DT_RAMP  = 0.1;                    % ramp point spacing [s]

% ★ Stick amplitude and speed are related quadratically (0.4208 → 3.234, 1.0 → 18.26 m/s). 0.5450 gives a measured 5.000 m/s.
if nargin < 2 || isempty(CR_AMP), CR_AMP = 0.5450; end   % calibrated value: 0.5450 → 5.000 m/s
T.CR_AMP = CR_AMP;
% ★ Yaw stick × 360 °/s is the commanded rate; the actual yaw rate is about 3% higher.
%   The amplitude is lowered to give an actual 45.0 °/s, and the hold time is extended to settle at 90.0°.
if nargin < 3 || isempty(YR_AMP), YR_AMP = 0.1214; end
if nargin < 4 || isempty(TURN_DUR), TURN_DUR = 2.06; end
T.YR_AMP = YR_AMP;

T.TK_T0  = 1.00;   T.TK_AMP = 0.05;    T.TK_DUR = 2.19;
T.HOV1   = 15.50;
T.EST    = [13.24, 14.54];
T.CL_AMP = 0.4208; T.CL_DUR = 6.25;
T.YAW0   = 0;                        % initial yaw 0° = facing north (read by basevars_IRIS)
T.TURN_T0 = 28.00 - T.CR_RAMP;       % turn start 26.33 (end of climb and settle)
T.TURN_RATE = 45;                    % turn rate [°/s]
T.TURN_DUR  = TURN_DUR;              % turn stick hold [s] — nominally 90°/45 °/s = 2 s, 2.06 s to cover the response lag
T.TURN_T1 = T.TURN_T0 + T.TURN_DUR;  % turn stick end 28.39
T.CR_T0  = T.TURN_T0 + 90 / T.TURN_RATE;   % start ramp begins 28.33 (overlaps the turn stick by 0.06 s, but the speed command starts at 0)
T.CR_ON  = T.CR_T0 + T.CR_RAMP;      % start ramp end = cruise start 30.00
T.TFAIL  = 35.00;
T.CR_OFF = 56.00;                    % stop ramp end = hover start
T.CR_T1  = T.CR_OFF - T.CR_RAMP;     % stop ramp start 54.33
T.HOV2   = 76.00;
T.TEND   = TEND;

T.ALT_TAKEOFF = T.TK_AMP*T.TK_DUR*T.K_ALT;
T.ALT_CRUISE  = T.ALT_TAKEOFF + T.CL_AMP*T.CL_DUR*T.K_ALT;

%% ── Stick time series ──
tk1 = T.TK_T0;  tk2 = tk1 + 0.05;  tk3 = T.TK_T0 + T.TK_DUR;  tk4 = tk3 + 0.05;
cl1 = T.HOV1;   cl2 = cl1 + 0.05;  cl3 = T.HOV1 + T.CL_DUR;   cl4 = cl3 + 0.05;

% Ramp points — values are held between points, so each point raises the value
r1 = (T.CR_T0 : T.DT_RAMP : T.CR_ON).';   a1 = CR_AMP * (r1 - r1(1)) / T.CR_RAMP;
r2 = (T.CR_T1 : T.DT_RAMP : T.CR_OFF).';  a2 = CR_AMP * (1 - (r2 - r2(1)) / T.CR_RAMP);

% Per-channel (time, value) event tables → common time base with hold. Channels = 1 east · 2 north (negative) · 3 yaw · 4 throttle
% ★ Horizontal sticks are in ground axes, not body axes (channel 1 = east, channel 2 = north (negative)).
%   After the turn the vehicle faces east, so channel 1 (+) is forward. A clockwise turn (north → east) is a positive yaw stick
EV = cell(1,4);
EV{1} = [0 0; r1, a1; r2, a2; T.CR_OFF 0; T.HOV2 0];   % zero point at the stop ramp end (56.0 s) — otherwise the last ramp value would persist to 76 s
if NOSTOP, EV{1} = [0 0; r1, a1; T.HOV2 CR_AMP]; end   % no stop ramp
EV{2} = [0 0];
EV{3} = [0 0; T.TURN_T0 YR_AMP; T.TURN_T1 0];
EV{4} = [0 0; tk1 0; tk2 T.TK_AMP; tk3 T.TK_AMP; tk4 0; cl1 0; cl2 T.CL_AMP; cl3 T.CL_AMP; cl4 0];
t = unique([0; TEND; cell2mat(cellfun(@(e) e(:,1), EV.', 'UniformOutput', false))]);
t = t(t <= TEND);
n = numel(t);  v = zeros(n, 4);
for k = 1:4
    e = EV{k};
    for i = 1:n
        j = find(e(:,1) <= t(i) + 1e-9, 1, 'last');   % hold previous value
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
