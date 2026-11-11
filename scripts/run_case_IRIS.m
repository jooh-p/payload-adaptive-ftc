function [r, d] = run_case_IRIS(ctx, kind, PM, fail, over)
%% run_case_IRIS — 제어기 한 판을 돌려 지표를 돌려준다
%
% ┌─ 무엇을 돌리나 ─────────────────────────────────────────────────────
% │ 가      무화물 · 기반 제어기 · 고장 있음          (참조선)
% │ 나      화물   · 기반 제어기 · 고장 있음          (무적응 기준선)
% │ 다      화물   · 감지 순간 파라미터 넷 전환       (제안)
% │ 무고장  화물   · 기반 제어기 · 고장 없음
% └─────────────────────────────────────────────────────────────────────
%
% 입력
%   ctx   ctx_current_IRIS() 의 꾸러미
%   kind  '가' | '나' | '다' | '무고장'
%   PM    화물 질량 [kg] (가는 0)
%   fail  고장 로터 번호 1~4 (기본 2)
%   over  (선택) mkctrl_current_IRIS 의 덮어쓰기 — .fsel · .dadd · .wy · .thr
%
% 출력 r
%   e 초기 강하 [m] · l 후반 최대 강하 [m] · tilt 후반 최대 기울기 [°] · reH 도달 오차 [m]
%   alive 생존 · mest 추정 질량 [kg] · fsel · dsel · tdet 감지 시각 [s]
%   무고장이면 crdev 순항 고도 편차 · crspd 순항 속도 · tiltmax
%
% 출력 d  (선택) 그 판의 시계열 — t · pn · pe · alt · te · tilt · tr · r
%   추종 오차처럼 시계열이 필요한 지표를 밖에서 계산할 때 쓴다
%
% 평가 창 값은 cfg_current_IRIS.

if nargin < 4 || isempty(fail), fail = 2; end
if nargin < 5, over = struct(); end
C = ctx.C;  m = ctx.m;  T = ctx.T;  meta = ctx.meta;
nofail = strcmp(kind, '무고장');
base   = any(strcmp(kind, {'가','나'}));
if strcmp(kind, '가'), PM = 0; end

%% ── 시뮬 변수 ──
V = basevars_IRIS(ctx.cdry, ctx.m_dry, meta, PM, C.POFF, T, C.TEND, ~nofail);
% ★ 플랜트 화물 물리 갈고리 — ctx.plantmode 가 있을 때만 동작한다.
%   'box'(기본)  상자 관성 = 위치 관성 + 상자 형상 관성
%   'point'      점질량 = 위치 관성만
%   'massonly'   질량만 늘리고 관성은 무화물 값 그대로
%   ★ 공력 갈고리보다 먼저 둘 것. 뒤에 두면 V.copter 를 통째로 바꿔 aero 를 되돌린다.
if isfield(ctx,'plantmode') && ~isempty(ctx.plantmode) && ~strcmp(ctx.plantmode,'box')
    switch ctx.plantmode
        case 'point'
            V.copter = mkplant_payload_IRIS(ctx.cdry, PM, C.POFF, false);
        case 'massonly'
            V.copter = ctx.cdry;
            V.copter.body.m = ctx.cdry.body.m + PM;    % 관성은 손대지 않는다
        otherwise
            error('run_case_IRIS:plantmode', '알 수 없는 plantmode: %s', ctx.plantmode);
    end
end
% ★ 플랜트 공력 덮어쓰기 갈고리 — ctx.plantaero 가 있을 때만 동작한다.
%   제어기는 ctx.cdry 로 따로 만들므로 여기 값은 **플랜트에만** 들어간다.
%   예: ctx.plantaero = struct('rate_damp', 0.60)
if isfield(ctx,'plantaero') && ~isempty(ctx.plantaero)
    fn = fieldnames(ctx.plantaero);
    for kk = 1:numel(fn), V.copter.aero.(fn{kk}) = ctx.plantaero.(fn{kk}); end
end
% ★ 실험 갈고리 — 셋 다 ctx 에 그 항목이 있을 때만 동작한다. 없으면 기존 경로 그대로.
%   ctx.plantpatch  {묶음, 항목, 값; ...} — 드론 모델에만 (예 {'prop','I',2*I0; 'bat','V',10.0})
%   ctx.vars        구조체 — 시뮬 변수를 더한다 (예 잡음 모델의 gyro_sigma · noise_seed)
%   ctx.blkpar      {블록 경로(모델 이름 뒤), 파라미터, 값; ...} — simseg_IRIS 가 블록 값으로 넘긴다
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

%% ── 가 · 나 — 기반 제어기 한 판 ──
if base
    V = applyctrl_IRIS(V, meta, F50, 0, 0);
    ob = simseg_IRIS(m, 0, C.TEND, [], V);
    d = pull_(ob);
    r = fill_(r, met_(d, ctx));
    if isfield(over,'keepsig') && over.keepsig, r.sig = sig_({ob}); end
    return
end

%% ── 1구간 — 호버와 질량 추정 ──
o1   = simseg_IRIS(m, 0, ctx.TSW, [], applyctrl_IRIS(V, meta, F50, 0, 0));
mest = NaN;
try
    e = o1.logsout.getElement('mest');  x = squeeze(e.Values.Data);  mest = x(end);
catch
end

%% ── 무고장 — 전환이 없으므로 기반 제어기로 끝까지 ──
if nofail
    d  = pull_(simseg_IRIS(m, 0, C.TEND, [], applyctrl_IRIS(V, meta, F50, 0, 0)));
    kc = d.t >= T.CR_ON & d.t <= T.CR_T1;
    ke = d.t >= T.CR_OFF & d.t <= C.W2(2);                       % 도달 오차 창(호버)
    r.crdev  = max(abs(d.alt(kc) - mean(d.alt(kc))));
    r.crspd  = mean(hypot(diff(d.pn(kc)), diff(d.pe(kc))) ./ max(diff(d.t(kc)), eps));   % 진행 방향 무관
    r.reH    = mean(hypot(d.pn(ke) - ctx.DEST(1), d.pe(ke) - ctx.DEST(2)));
    r.tiltmax = max(d.tilt);  r.tilt = r.tiltmax;
    r.e = 0;  r.l = 0;  r.alive = true;  r.mest = mest;
    return
end

%% ── 2구간 — 정상 비행, 감지되는 순간 모델이 스스로 정지 ──
o2   = simseg_IRIS(m, ctx.TSW, C.TEND, o1.opPt, msk_(applyctrl_IRIS(V, meta, F50, 0, 1), over));
tdet = o2.tout(end);

%% ── 3구간 — 감지 뒤, 파라미터 넷을 전환 ──
[f3, sel] = mkctrl_current_IRIS('post', ctx.cdry, C.POFF, mest, over);
% ★ 감지 지연 갈고리 (over.tdelay [s]) — 감지 뒤 tdelay 동안 기반 제어기 · 주입 0 으로 더 날고 전환한다.
%   감지 차트(FFDetect)의 주입 타이머는 감지 순간부터 흐르므로 지연 구간과 전환 뒤 모두 nff(ffpar(4))에
%   지연 표본 수를 더해, 주입이 tdet + tdelay 부터 원래 길이만큼 들어가게 한다. 없거나 0 이면 기존 경로 그대로.
tdl = 0;  if isfield(over,'tdelay') && ~isempty(over.tdelay), tdl = over.tdelay; end
% 지연이 제어 표본 반 개(1.25 ms = 시뮬 간격) 미만이면 nd = 0 이라 주입 타이머도 그대로다 — 지연 없음으로 본다
% 구간이 시뮬 간격보다 짧으면 "stop time ≤ snapshot time" 오류가 나므로 이 경우를 거른다.
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
    d  = splice_(pull_(o1), pull_(o2));        % 감지 못 함
end

r = fill_(r, met_(d, ctx));
r.mest = mest;  r.dsel = sel.dsel;  r.fsel = sel.fsel;  r.tdet = tdet;
% ★ 신호 보관 갈고리 (over.keepsig) — 지표 경로는 그대로 두고 구간별 원 신호만 이어 붙여 r.sig 로 돌려준다.
if isfield(over,'keepsig') && over.keepsig
    sg = {o1, o2};
    if exist('o2d','var'), sg{end+1} = o2d; end
    if exist('o3','var'),  sg{end+1} = o3;  end
    r.sig = sig_(sg);
end
end

function s = sig_(sg)
%% 원 신호 이어 붙이기 — 위치 · 오일러각 · 몸체 각속도 · 모터 회전수 (겹치는 앞부분은 버린다)
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

%% ══════════════ 도우미 ══════════════
function v = met_(d, ctx)
%% 지표 — 평가 창은 절대 시각으로 고정
    T = ctx.T;  C = ctx.C;
    a0 = interp1(d.t, d.alt, T.TFAIL, 'linear', d.alt(1));       % 고장 시각 고도
    s1 = d.t  >= T.TFAIL & d.t  <= T.TFAIL + C.EARLY;            % 초기 창
    s2 = d.t  >= C.W2(1) & d.t  <= C.W2(2);                      % 후반 창
    t2 = d.te >= C.W2(1) & d.te <= C.W2(2);
    v.e    = a0 - min(d.alt(s1));
    v.l    = a0 - min(d.alt(s2));
    v.tilt = max(d.tilt(t2));
    sr     = d.t >= T.CR_OFF & d.t <= C.W2(2);                   % 도달 오차 창(호버) — 후반 창과 분리
    v.reH  = mean(hypot(d.pn(sr) - ctx.DEST(1), d.pe(sr) - ctx.DEST(2)));
    v.alive = (a0 - min(d.alt(d.t >= T.TFAIL))) <= 45 && v.reH <= 5;
end

function r = fill_(r, q)
    f = fieldnames(q);
    for k = 1:numel(f), r.(f{k}) = q.(f{k}); end
end

function d = pull_(o)
%% 로그에서 시계열을 꺼낸다
    t = o.s_g.Time(:).';  P = squeeze(o.s_g.Data);  if size(P,1) ~= 3, P = P.'; end
    d.t = t;  d.pn = P(1,:);  d.pe = P(2,:);  d.alt = -P(3,:);
    te = o.Euler_angles.Time(:).';  E = squeeze(o.Euler_angles.Data);  if size(E,1) ~= 3, E = E.'; end
    d.te = te;  d.tilt = acosd(max(-1, min(1, cos(E(1,:)).*cos(E(2,:)))));
    d.tr = o.omega_Kb.Time(:).';  W = squeeze(o.omega_Kb.Data);  if size(W,1) ~= 3, W = W.'; end
    d.r = W(3,:)*180/pi;
end

function d = splice_(varargin)
%% 구간 셋을 이어 붙인다 (겹치는 앞부분은 버린다)
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
%% 주입 대각 쌍 강제 — over.mask = [1 0 1 0] 이면 M1 · M3, [0 1 0 1] 이면 M2 · M4.
%  없으면 기존 경로(cfg 의 mask = 0, 감지기가 요 오차 부호로 고름). 감지 순간(2구간)부터 같은 값을 넣어야 한다.
    if isfield(over, 'mask') && ~isempty(over.mask), V.ffpar(8:11) = over.mask(:); end
end

function y = tern_(c, a, b), if c, y = a; else, y = b; end, end
