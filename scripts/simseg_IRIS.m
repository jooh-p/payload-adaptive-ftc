function o = simseg_IRIS(m, t0, t1, op, V)
%% simseg_IRIS — run one time segment
%
% Variables are passed through SimulationInput instead of the base workspace,
% so this also works inside parfor. Build the variable struct with basevars_IRIS
% and add the controller and feed-forward thrust with applyctrl_IRIS.
%
% Inputs
%   m   model name · t0/t1 segment [s] · op initial operating point ([] = start from scratch)
%   V   variable struct
% Output
%   o   simulation output (operating point for the next segment is o.opPt)

in = Simulink.SimulationInput(m);
in = in.setModelParameter('StartTime', num2str(t0), 'StopTime', num2str(t1));
in = in.setModelParameter('SaveFinalState','on', 'SaveOperatingPoint','on', ...
                          'FinalStateName','opPt');
fn = setdiff(fieldnames(V), {'blkpar'}, 'stable');
for i = 1:numel(fn), in = in.setVariable(fn{i}, V.(fn{i})); end
% Optional block parameter override — V.blkpar = {path (after model name), parameter, value; ...}. The model file is not changed.
if isfield(V, 'blkpar')
    for i = 1:size(V.blkpar,1), in = in.setBlockParameter([m '/' V.blkpar{i,1}], V.blkpar{i,2}, V.blkpar{i,3}); end
end
if ~isempty(op), in = in.setInitialState(op); end
evalc('o = sim(in);');
end
