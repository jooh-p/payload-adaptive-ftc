function V = applyctrl_IRIS(V, meta, fm, delta, stopen)
%% applyctrl_IRIS — put the controller and feed-forward thrust parameters into the sim variable struct
%
% Feed-forward thrust duration and injection mask are read from cfg_current_IRIS.
%
% Inputs
%   V       sim variable struct made by basevars_IRIS
%   meta    meta of abc_IRIS.mat (control period TS · detection threshold TH · persistence PERSIST · RMAX · RTC)
%   fm      controller (mkctrl_current_IRIS)
%   delta   feed-forward thrust magnitude (0 = no injection)
%   stopen  1 = the model stops itself at the detection instant (segmented run)

C  = cfg_current_IRIS();
ts = meta.TS;
V.fm_loiter = fm;
V.ffpar = [delta; meta.TH; max(1, round(meta.PERSIST/ts)); max(1, round(C.tau/ts)); ...
           meta.RMAX; meta.RTC; ts; C.mask(:)];
V.dseg_stop_en = stopen;
end
