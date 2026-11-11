function [r, d] = run_case_IRIS(ctx, kind, PM, fail, over)
%% run_case_IRIS — run one case and return its metrics
%
% ┌─ Cases ─────────────────────────────────────────────────────────────
% │ 가      no payload · baseline controller · failure        (reference)
% │ 나      payload    · baseline controller · failure        (non-adaptive baseline)
% │ 다      payload    · parameters switched at detection     (proposed)
% │ 무고장  payload    · baseline controller · no failure
% └─────────────────────────────────────────────────────────────────────
%
% Inputs
%   ctx   context from ctx_current_IRIS()
%   kind  '가' | '나' | '다' | '무고장'
%   PM    payload mass [kg] (0 for 가)
%   fail  failed rotor number 1–4 (default 2)
%   over  (optional) overrides for mkctrl_current_IRIS — .fsel · .dadd · .wy · .thr
%
% Output r
%   e initial drop [m] · l late-phase max drop [m] · tilt late-phase max tilt [°] · reH reach error [m]
%   alive survived · mest estimated mass [kg] · fsel · dsel · tdet detection time [s]
%   for 무고장: crdev cruise altitude deviation · crspd cruise speed · tiltmax
%
% Output d  (optional) time series of the case — t · pn · pe · alt · te · tilt · tr · r
%   for computing time-series metrics (e.g. tracking error) outside this function
%
% Evaluation window values are in cfg_current_IRIS.

if nargin < 4 || isempty(fail), fail = 2; end
if nargin < 5, over = struct(); end
C = ctx.C;  m = ctx.m;  T = ctx.T;  meta = ctx.meta;
nofail = strcmp(kind, '무고장');
base   = any(strcmp(kind, {'가','나'}));
if strcmp(kind, '가'), PM = 0; end

%% ── Simulation variables ──
V = basevars_IRIS(ctx.cdry, ctx.m_dry, meta, PM, C.POFF, T, C.TEND, ~nofail);
% ★ Payload physics hook — active only if ctx.plantmode is set.
%   'box' (default)  box inertia = offset inertia + box shape inertia
%   'point'          point mass = offset inertia only
%   'massonly'       mass increased, inertia kept at payload-free values
%   ★ Must come before the aero hook; placed after, it replaces V.copter and undoes aero.
if isfield(ctx,'plantmode') && ~isempty(ctx.plantmode) && ~strcmp(ctx.plantmode,'box')
    switch ctx.plantmode
        case 'point'
            V.copter = mkplant_payload_IRIS(ctx.cdry, PM, C.POFF, false);
        case 'massonly'
            V.copter = ctx.cdry;
            V.copter.body.m = ctx.cdry.body.m + PM;    % inertia not changed
        otherwise
            error('run_case_IRIS:plantmode', '알 수 없는 plantmode: %s', ctx.plantmode);
    end
end
% ★ Drone-model aero override hook — active only if ctx.plantaero is set.
%   The controller is built separately from ctx.cdry, so these values go into the drone model only.
%   e.g. ctx.plantaero = struct('rate_damp', 0.60)
if isfield(ctx,'plantaero') && ~isempty(ctx.plantaero)
    fn = fieldnames(ctx.plantaero);
    for kk = 1:numel(fn), V.copter.aero.(fn{kk}) = ctx.plantaero.(fn{kk}); end
end
% ★ Experiment hooks — each is active only if the field exists in ctx; otherwise the default path is used.
%   ctx.plantpatch  {group, field, value; ...} — drone model only (e.g. {'prop','I',2*I0; 'bat','V',10.0})
%   ctx.vars        struct — adds simulation variables (e.g. gyro_sigma · noise_seed of a noise model)
%   ctx.blkpar      {block path (after model name), parameter, value; ...} — passed to simseg_IRIS as block values
if isfield(ctx,'plantpatch') && ~isempty(ctx.plantpatch)
    for kk = 1:size(ctx.plantpatch,1)
        V.copter.(ctx.plantpatch{kk,1}).(ctx.plantpatch{kk,2}) = ctx.plantpatch{kk,3};
    end
end
if isfield(ctx,'vars') && ~isempty(ctx.vars)
    fn = fieldnames(ctx.vars);
    for kk = 1:numel(fn), V.(fn{kk}) = ctx.vars.(fn{kk}); end
end
if isfield(ctx,'blkpar') && ~isempty(ctx.blkpar), V.blkpar = ctx.blkpar; end
V.simin = ctx.simin;
if ~nofail
    for k = 1:4
        V.(sprintf('failure_time_mot_%d', k)) = tern_(k == fail, T.TFAIL, 1000);
    end
end

F50 = mkctrl_current_IRIS('base', ctx.cdry, C.POFF);
r   = struct('mest',NaN, 'fsel',NaN, 'dsel',NaN, 'tdet',NaN, ...
             'crdev',NaN, 'crspd',NaN, 'tiltmax',NaN);

%% ── 가 · 나 — one baseline-controller run ──
if base
    V = applyctrl_IRIS(V, meta, F50, 0, 0);
    ob = simseg_IRIS(m, 0, C.TEND, [], V);
    d = pull_(ob);
    r = fill_(r, met_(d, ctx));
    if isfield(over,'keepsig') && over.keepsig, r.sig = sig_({ob}); end
    return
end

%% ── Segment 1 — hover and mass estimation ──
o1   = simseg_IRIS(m, 0, ctx.TSW, [], applyctrl_IRIS(V, meta, F50, 0, 0));
mest = NaN;
try
    e = o1.logsout.getElement('mest');  x = squeeze(e.Values.Data);  mest = x(end);
catch
end

%% ── 무고장 — no switching, baseline controller to the end ──
if nofail
    d  = pull_(simseg_IRIS(m, 0, C.TEND, [], applyctrl_IRIS(V, meta, F50, 0, 0)));
    kc = d.t >= T.CR_ON & d.t <= T.CR_T1;
    ke = d.t >= T.CR_OFF & d.t <= C.W2(2);                       % reach-error window (hover)
    r.crdev  = max(abs(d.alt(kc) - mean(d.alt(kc))));
    r.crspd  = mean(hypot(diff(d.pn(kc)), diff(d.pe(kc))) ./ max(diff(d.t(kc)), eps));   % independent of heading
    r.reH    = mean(hypot(d.pn(ke) - ctx.DEST(1), d.pe(ke) - ctx.DEST(2)));
    r.tiltmax = max(d.tilt);  r.tilt = r.tiltmax;
    r.e = 0;  r.l = 0;  r.alive = true;  r.mest = mest;
    return
end

%% ── Segment 2 — normal flight; the model stops itself at detection ──
o2   = simseg_IRIS(m, ctx.TSW, C.TEND, o1.opPt, msk_(applyctrl_IRIS(V, meta, F50, 0, 1), over));
tdet = o2.tout(end);

%% ── Segment 3 — after detection, switch the parameters ──
[f3, sel] = mkctrl_current_IRIS('post', ctx.cdry, C.POFF, mest, over);
% ★ Detection delay hook (over.tdelay [s]) — fly tdelay longer with the baseline controller and no injection, then switch.
%   The injection timer of the detection chart (FFDetect) starts at detection, so the delay samples are added to
%   nff (ffpar(4)) in both the delay and post-switch segments so injection runs its full length from tdet + tdelay. Absent or 0 = default path.
tdl = 0;  if isfield(over,'tdelay') && ~isempty(over.tdelay), tdl = over.tdelay; end
% A delay below half a control sample (1.25 ms = sim step) gives nd = 0, so it is treated as no delay;
% a segment shorter than the sim step would raise a "stop time ≤ snapshot time" error.
if round(tdl / meta.TS) == 0, tdl = 0; end
if tdl > 0 && tdet < C.TEND - 1e-6
    nd  = round(tdl / meta.TS);
    Vd  = msk_(applyctrl_IRIS(V, meta, F50, 0, 0), over);  Vd.ffpar(4) = Vd.ffpar(4) + nd;
    tsw = min(tdet + tdl, C.TEND);
    o2d = simseg_IRIS(m, tdet, tsw, o2.opPt, Vd);
    V3  = msk_(applyctrl_IRIS(V, meta, f3, sel.dsel, 0), over);  V3.ffpar(4) = V3.ffpar(4) + nd;
    if tsw < C.TEND - 1e-6
        o3 = simseg_IRIS(m, tsw, C.TEND, o2d.opPt, V3);
        d  = splice_(pull_(o1), pull_(o2), pull_(o2d), pull_(o3));
    else
        d  = splice_(pull_(o1), pull_(o2), pull_(o2d));
    end
elseif tdet < C.TEND - 1e-6
    o3 = simseg_IRIS(m, tdet, C.TEND, o2.opPt, msk_(applyctrl_IRIS(V, meta, f3, sel.dsel, 0), over));
    d  = splice_(pull_(o1), pull_(o2), pull_(o3));
else
    d  = splice_(pull_(o1), pull_(o2));        % not detected
end

r = fill_(r, met_(d, ctx));
r.mest = mest;  r.dsel = sel.dsel;  r.fsel = sel.fsel;  r.tdet = tdet;
% ★ Signal keep hook (over.keepsig) — metric path unchanged; raw signals of all segments are spliced into r.sig.
if isfield(over,'keepsig') && over.keepsig
    sg = {o1, o2};
    if exist('o2d','var'), sg{end+1} = o2d; end
    if exist('o3','var'),  sg{end+1} = o3;  end
    r.sig = sig_(sg);
end
end

function s = sig_(sg)
%% splice raw signals — position · Euler angles · body rates · motor speeds (overlapping starts are dropped)
    nm = {'s_g','Euler_angles','omega_Kb','motor_speed'};
    for j = 1:numel(nm)
        t = [];  X = [];
        for i = 1:numel(sg)
            ts = sg{i}.(nm{j});  ti = ts.Time(:).';  Xi = squeeze(ts.Data);
            if size(Xi,2) ~= numel(ti), Xi = Xi.'; end
            if ~isempty(t), k = ti > t(end); else, k = true(size(ti)); end
            t = [t ti(k)];  X = [X Xi(:,k)]; %#ok<AGROW>
        end
        s.(nm{j}) = struct('t', t, 'X', X);
    end
end

%% ══════════════ helpers ══════════════
function v = met_(d, ctx)
%% metrics — evaluation windows are fixed in absolute time
    T = ctx.T;  C = ctx.C;
    a0 = interp1(d.t, d.alt, T.TFAIL, 'linear', d.alt(1));       % altitude at failure time
    s1 = d.t  >= T.TFAIL & d.t  <= T.TFAIL + C.EARLY;            % initial window
    s2 = d.t  >= C.W2(1) & d.t  <= C.W2(2);                      % late-phase window
    t2 = d.te >= C.W2(1) & d.te <= C.W2(2);
    v.e    = a0 - min(d.alt(s1));
    v.l    = a0 - min(d.alt(s2));
    v.tilt = max(d.tilt(t2));
    sr     = d.t >= T.CR_OFF & d.t <= C.W2(2);                   % reach-error window (hover) — separate from the late-phase window
    v.reH  = mean(hypot(d.pn(sr) - ctx.DEST(1), d.pe(sr) - ctx.DEST(2)));
    v.alive = (a0 - min(d.alt(d.t >= T.TFAIL))) <= 45 && v.reH <= 5;
end

function r = fill_(r, q)
    f = fieldnames(q);
    for k = 1:numel(f), r.(f{k}) = q.(f{k}); end
end

function d = pull_(o)
%% extract time series from the logs
    t = o.s_g.Time(:).';  P = squeeze(o.s_g.Data);  if size(P,1) ~= 3, P = P.'; end
    d.t = t;  d.pn = P(1,:);  d.pe = P(2,:);  d.alt = -P(3,:);
    te = o.Euler_angles.Time(:).';  E = squeeze(o.Euler_angles.Data);  if size(E,1) ~= 3, E = E.'; end
    d.te = te;  d.tilt = acosd(max(-1, min(1, cos(E(1,:)).*cos(E(2,:)))));
    d.tr = o.omega_Kb.Time(:).';  W = squeeze(o.omega_Kb.Data);  if size(W,1) ~= 3, W = W.'; end
    d.r = W(3,:)*180/pi;
end

function d = splice_(varargin)
%% splice segments (overlapping starts are dropped)
    d = varargin{1};
    f = {'t','pn','pe','alt'};  g = {'te','tilt'};
    for i = 2:numel(varargin)
        a  = varargin{i};
        k  = a.t  > d.t(end);   for j = 1:numel(f), d.(f{j}) = [d.(f{j}) a.(f{j})(k)];  end
        k2 = a.te > d.te(end);  for j = 1:numel(g), d.(g{j}) = [d.(g{j}) a.(g{j})(k2)]; end
        k3 = a.tr > d.tr(end);  d.tr = [d.tr a.tr(k3)];  d.r = [d.r a.r(k3)];
    end
end

function V = msk_(V, over)
%% force the injection diagonal pair — over.mask = [1 0 1 0] gives M1 · M3, [0 1 0 1] gives M2 · M4.
%  Absent = default path (cfg mask = 0, the detector picks by the sign of the yaw error). Must be set from detection (segment 2) on.
    if isfield(over, 'mask') && ~isempty(over.mask), V.ffpar(8:11) = over.mask(:); end
end

function y = tern_(c, a, b), if c, y = a; else, y = b; end, end
