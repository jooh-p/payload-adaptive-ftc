function [fm, sel] = mkctrl_current_IRIS(stage, cdry, POFF, mest, over)
%% mkctrl_current_IRIS — 현행 제어기를 만든다 (2026-09-22 신설, 단일 출처)
%
% ┌─ 왜 ────────────────────────────────────────────────────────────────
% │ 제어기 생성(무화물 몸통 · 위치 게인 배율 · 두 배분 가중)이 symfinal_IRIS 와
% │ fig3_abc4_IRIS 에 같은 이름의 지역 함수로 복제돼 있었다. 값은 cfg_current_IRIS.
% └─────────────────────────────────────────────────────────────────────
%
% 입력
%   stage  'base' — 기반 제어기(가·나, 그리고 다의 감지 전 구간)
%          'post' — 감지 뒤 다 (위치 게인 배율 + 요·추력 배분 가중)
%   cdry   무화물 기체 구조체 (대칭 제원이 적용된 것)
%   POFF   화물 장착 깊이 [m]
%   mest   추정 화물 질량 [kg] — 'post' 에서 표 보간에 씀. 'base' 는 무시
%   over   (선택) 표 대신 쓸 값 — .fsel 위치 배율 · .dsel 선제 추력(절대) · .dadd 증감 · .wy · .thr
%          .bodyPM  제어기 내부 모델의 질량에 더할 값 [kg] (0-H 실험 전용, 2026-09-28).
%                   true 이면 mest(추정 질량)를 쓴다. 관성(body.I)은 건드리지 않는다.
%                   ★ 규약(6) "몸통은 언제나 무화물" 의 예외이며, 'post' 단계에서만 적용된다.
%
% 출력
%   fm     제어기 구조체
%   sel    .fsel 위치 배율 · .dsel 선제 추력 크기 · .wy · .thr (실제 적용값)
%
% 쓰는 법
%   [F50, ~]   = mkctrl_current_IRIS('base', cdry, POFF);
%   [f3, sel]  = mkctrl_current_IRIS('post', cdry, POFF, mest);

C = cfg_current_IRIS();
if nargin < 4, mest = NaN; end
if nargin < 5, over = struct(); end
g = @(f, dflt) getopt_(over, f, dflt);   % ★ 익명 함수 안에서 인자를 먼저 계산하므로 실제 함수로 뺀다

%% ── 무화물 몸통 (세 제어기 공통) ──
copter = mkplant_payload_IRIS(cdry, 0, POFF, false); %#ok<NASGU>   % ★ 변수명 반드시 copter
bodyPM = g('bodyPM', 0);                                            % 0-H 전용: 내부 모델 질량 갱신
if strcmp(stage, 'post') && ~isequal(bodyPM, 0)
    if islogical(bodyPM) || ischar(bodyPM), bodyPM = mest; end      % true / 'mest' → 추정 질량
    if isfinite(bodyPM), copter.body.m = copter.body.m + bodyPM; end   % 질량만, 관성은 무화물 그대로
end
fm = lindiCopterAutoCreate(copter, 'AgilityAtti', C.aA, 'AgilityPos', C.aP, ...
                           'FilterStrength', 0, 'CntrlEffectScaling', 1);
fm.psc.inditype = 1;

sel = struct('fsel', 1, 'dsel', 0, 'wy', 1, 'thr', 1);
if strcmp(stage, 'base'), return; end

%% ── 감지 뒤 — 파라미터 넷 ──
mest = mest * g('mscale', 1);   % ★ 심사 대응 실험 2 (2026-10-28) — 추정 질량에 오차 배수(1+ε)를 곱해 표를 읽는다. 기본 1 = 기존 경로
mc = @(x) min(max(x, C.PMS(1)), C.PMS(end));                        % 표 범위 안으로 자름
sel.fsel = g('fsel', interp1(C.PMS, C.tabF, mc(mest), 'linear'));   % 위치 게인 배율
sel.dsel = g('dsel', interp1(C.PMS, C.injD, mc(mest), 'linear') + g('dadd', 0));  % 선제 추력 크기 (dsel 은 절대값 덮어쓰기)
sel.wy   = g('wy',  C.wy);                                          % 요 배분 가중 배수
sel.thr  = g('thr', C.thr);                                         % 추력 배분 가중 배수

fm.psc.k.pos   = fm.psc.k.pos * sel.fsel;
fm.ca.W_v(3,3) = fm.ca.W_v(3,3) * sel.wy;
fm.ca.W_v(4,4) = fm.ca.W_v(4,4) * sel.thr;
end

function v = getopt_(s, f, dflt)
%% 구조체에 그 항목이 있으면 그 값을, 없으면 기본값을 준다
    if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = dflt; end
end
