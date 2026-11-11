function [fm, sel] = mkctrl_current_IRIS(stage, cdry, POFF, mest, over)
%% mkctrl_current_IRIS — build the controller
%
% Builds the baseline controller (for the payload-free airframe). In the 'post' stage it
% interpolates the parameter schedule with the estimated mass and switches the position
% gain scale, yaw control allocation weight and thrust allocation weight.
% Values come from cfg_current_IRIS.
%
% Inputs
%   stage  'base' — baseline controller (가 · 나, and 다 before detection)
%          'post' — 다 after detection (position gain scale + yaw/thrust allocation weights)
%   cdry   payload-free airframe struct (with the symmetric layout applied)
%   POFF   payload mounting depth [m]
%   mest   estimated payload mass [kg] — used for table interpolation in 'post'; ignored in 'base'
%   over   (optional) values replacing the table — .fsel position scale · .dsel feed-forward thrust (absolute) · .dadd offset · .wy · .thr
%          .bodyPM  value added to the controller's internal model mass [kg] ('post' only, default 0)
%                   true = use mest (estimated mass). Inertia (body.I) is not changed.
%          .mscale  multiplier on the estimated mass (default 1)
%
% Outputs
%   fm     controller struct
%   sel    .fsel position scale · .dsel feed-forward thrust magnitude · .wy · .thr (values actually applied)
%
% Usage
%   [F50, ~]   = mkctrl_current_IRIS('base', cdry, POFF);
%   [f3, sel]  = mkctrl_current_IRIS('post', cdry, POFF, mest);

C = cfg_current_IRIS();
if nargin < 4, mest = NaN; end
if nargin < 5, over = struct(); end
g = @(f, dflt) getopt_(over, f, dflt);   % ★ anonymous functions evaluate arguments first, so use a real function

%% ── Payload-free body (common to all three controllers) ──
copter = mkplant_payload_IRIS(cdry, 0, POFF, false); %#ok<NASGU>   % ★ the variable must be named copter
bodyPM = g('bodyPM', 0);                                            % internal model mass correction (default 0 = off)
if strcmp(stage, 'post') && ~isequal(bodyPM, 0)
    if islogical(bodyPM) || ischar(bodyPM), bodyPM = mest; end      % true / 'mest' → estimated mass
    if isfinite(bodyPM), copter.body.m = copter.body.m + bodyPM; end   % mass only, inertia stays payload-free
end
fm = lindiCopterAutoCreate(copter, 'AgilityAtti', C.aA, 'AgilityPos', C.aP, ...
                           'FilterStrength', 0, 'CntrlEffectScaling', 1);
fm.psc.inditype = 1;

sel = struct('fsel', 1, 'dsel', 0, 'wy', 1, 'thr', 1);
if strcmp(stage, 'base'), return; end

%% ── After detection — parameter switching ──
mest = mest * g('mscale', 1);   % estimated-mass multiplier (default 1)
mc = @(x) min(max(x, C.PMS(1)), C.PMS(end));                        % clip to the table range
sel.fsel = g('fsel', interp1(C.PMS, C.tabF, mc(mest), 'linear'));   % position gain scale
sel.dsel = g('dsel', interp1(C.PMS, C.injD, mc(mest), 'linear') + g('dadd', 0));  % feed-forward thrust magnitude (dsel overrides with an absolute value)
sel.wy   = g('wy',  C.wy);                                          % yaw control allocation weight multiplier
sel.thr  = g('thr', C.thr);                                         % thrust allocation weight multiplier

fm.psc.k.pos   = fm.psc.k.pos * sel.fsel;
fm.ca.W_v(3,3) = fm.ca.W_v(3,3) * sel.wy;
fm.ca.W_v(4,4) = fm.ca.W_v(4,4) * sel.thr;
end

function v = getopt_(s, f, dflt)
%% return the field value if present, otherwise the default
    if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = dflt; end
end
