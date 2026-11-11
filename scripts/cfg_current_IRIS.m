function C = cfg_current_IRIS()
%% cfg_current_IRIS — single place for all experiment settings
%
% Defines the model, scenario, evaluation windows, airframe data, parameter schedule
% and mass-independent constants. All other scripts read these values only.
%
% Usage:  C = cfg_current_IRIS();

%% ── Model and scenario ──
C.model = 'QuadcopterSimModel_Loiter_FTC_dseg';   % experiment model
C.scn   = 'scn_delivery3_IRIS';                   % delivery scenario (90° turn 26.33–28.33 s · failure 35.00 s · end 77 s)
C.TEND  = 77;
%   Evaluation windows — initial drop [35, 45] s · late phase [45, 77] s
%   45 s is when the baseline controller (no payload) first recovers altitude after the initial drop.
C.EARLY = 10.0;                                   % initial-drop window length [s] — [0, 10] after failure
C.W2    = [45 77];                                % late-phase window [s] (absolute time)

%% ── Target airframe (symmetric rotor layout) ──
%   Overwrites copter_params_IRIS after loading.
%   ★ Must be applied after copterLoadParams (applying it before is undone by the load)
C.propY   = [-0.21 0.21 0.21 -0.21];              % propeller y [m] — M1 M2 M3 M4
C.inertia = [0.021950 0.011 0.030949];            % diagonal inertia [kg·m²]
C.POFF    = 0.10;                                 % payload mounting depth [m]

%% ── Mass-estimation reference ──
%   Sum of squared motor speeds in a settled payload-free hover (data/symcheck_IRIS.mat)
C.sumsqFile = 'symcheck_IRIS.mat';                % in data/
C.sumsqVar  = 'sumsq_new';
C.sumsqChk  = 742700.6917;                        % check value of the file contents

%% ── Parameter schedule ──
%   Optimal values per design mass. The estimated mass is clipped to the range and
%   linearly interpolated between neighbouring design masses.
%   The three lines below (PMS · injD · tabF) are redefined in 'Current schedule'.
C.PMS  = [0 0.100 0.200 0.300 0.340];             % schedule design masses [kg]
C.injD = [0.6155 0.6055 0.5612 0.4865 0.4484];    % feed-forward thrust magnitude (normalized command)
C.tabF = [5.254 4.50 3.790 2.446 1.971];          % position gain scale
%% ── Current schedule (values in the paper) ──
%   Optimal values for design masses 100 · 200 · 300 · 340 g found by Bayesian adaptive direct search.
C.PMS  = [0.100 0.200 0.300 0.340];
C.tabF = [4.543977183 4.166586273 2.500473928 2.222778988];
C.injD = [0.6014710808 0.5622518154 0.5113042045 0.4856299388];
C.Jfloor = 0.64;                                 % floor of the late-phase term in the objective [m] — search objective only

%% ── Mass-independent constants ──
C.wy   = 253.7235;      % yaw control allocation weight multiplier
C.thr  = 3.994963809;   % thrust allocation weight multiplier
C.tau  = 1.80;          % feed-forward thrust duration [s]
C.mask = [0 0 0 0];     % 0 = the detector picks the diagonal pair from the sign of the yaw error (independent of the failed rotor)
C.aA   = 0.5;           % attitude agility — baseline controller default
C.aP   = 0.5;           % position agility — baseline controller default

%% ── Reach-error reference point (mission-failure check only, 5 m threshold) ──
%   Last sample position of the baseline controller (no payload) + failure run (127.9 m east).
C.destFile = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'ga_ref_dlv3_turn.mat');
C.destVar  = 'DEST';

%% ── Objective (paper Eq. (4)) ──
C.Jw   = [0.5 0.5 0.05];       % weights: initial drop · late-phase max drop · late-phase max tilt
C.Jref = [5.15 0.64 20.7];     % payload-free reference values [m, m, °]
%% ── Schedule override (optional) ──
%   Evaluates with the schedule in struct S (PMS · tabF · injD · wy · thr) of the mat file
%   given by environment variable CFG_SCHED_FILE. Does nothing if the variable is not set.
SF = getenv('CFG_SCHED_FILE');
if ~isempty(SF) && isfile(SF)
    Z = load(SF);  S = Z.S;
    C.PMS = S.PMS;  C.tabF = S.tabF;  C.injD = S.injD;  C.wy = S.wy;  C.thr = S.thr;
end
end
