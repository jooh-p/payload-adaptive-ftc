function ctx = ctx_current_IRIS()
%% ctx_current_IRIS — build a ready-to-simulate context
%
% Opens the model and prepares the airframe data, mass-estimation reference,
% delivery scenario and reach-error reference point. Values come from cfg_current_IRIS.
%
% Usage
%   ctx = ctx_current_IRIS();
%   r   = run_case_IRIS(ctx, '다', 0.340, 2);

C    = cfg_current_IRIS();
HERE = fileparts(mfilename('fullpath'));
DAT  = fullfile(HERE, '..', 'data');

%% ── Model ──
m = C.model;
if ~bdIsLoaded(m), load_system(m); end
set_param(m, 'UnconnectedInputMsg', 'none');
set_param(m, 'MultiTaskRateTransMsg', 'warning');
set_param(m, 'EnablePacing', 'off');          % pacing does not affect physics, it only slows the run

%% ── Target airframe ──
%   ★ Overwrite after copterLoadParams — doing it before is undone by the load
cdry = copterLoadParams('copter_params_IRIS');
cdry.config.propPos_c(2,:) = C.propY;
cdry.body.I                = diag(C.inertia);

%% ── Detection/injection constants and mass-estimation reference ──
S    = load('abc_IRIS.mat');  meta = S.ABC.meta;   % only meta (detection constants · control period) is used
Q    = load(fullfile(DAT, C.sumsqFile));
meta.sumsq_dry = Q.(C.sumsqVar);
meta.MASK      = C.mask;

%% ── Scenario and reach-error reference point ──
[simin, T] = feval(C.scn, C.TEND);
G = load(C.destFile);

ctx = struct('C', C, 'm', m, 'cdry', cdry, 'm_dry', cdry.body.m, 'meta', meta, ...
             'simin', simin, 'T', T, 'TSW', T.HOV1 - 0.5, 'DEST', G.(C.destVar));
end
