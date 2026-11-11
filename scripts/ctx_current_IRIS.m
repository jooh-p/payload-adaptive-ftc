function ctx = ctx_current_IRIS()
%% ctx_current_IRIS — 시뮬 준비를 끝낸 꾸러미를 만든다
%
% 모델 열기, 대상 기체 제원, 질량 추정 기준값, 배송 시나리오, 도달 오차 기준점을
% 한 번에 준비한다. 값은 cfg_current_IRIS 에서 읽는다.
%
% 쓰는 법
%   ctx = ctx_current_IRIS();
%   r   = run_case_IRIS(ctx, '다', 0.340, 2);

C    = cfg_current_IRIS();
HERE = fileparts(mfilename('fullpath'));
DAT  = fullfile(HERE, '..', 'data');

%% ── 모델 ──
m = C.model;
if ~bdIsLoaded(m), load_system(m); end
set_param(m, 'UnconnectedInputMsg', 'none');
set_param(m, 'MultiTaskRateTransMsg', 'warning');
set_param(m, 'EnablePacing', 'off');          % 페이싱은 물리와 무관하고 속도만 늦춘다

%% ── 대상 기체 ──
%   ★ copterLoadParams 뒤에 덮어쓸 것 — 앞에 두면 로드가 되돌린다
cdry = copterLoadParams('copter_params_IRIS');
cdry.config.propPos_c(2,:) = C.propY;
cdry.body.I                = diag(C.inertia);

%% ── 감지·주입 상수와 질량 추정 기준값 ──
S    = load('abc_IRIS.mat');  meta = S.ABC.meta;   % meta(감지 상수 · 제어 주기)만 쓴다
Q    = load(fullfile(DAT, C.sumsqFile));
meta.sumsq_dry = Q.(C.sumsqVar);
meta.MASK      = C.mask;

%% ── 시나리오와 도달 오차 기준점 ──
[simin, T] = feval(C.scn, C.TEND);
G = load(C.destFile);

ctx = struct('C', C, 'm', m, 'cdry', cdry, 'm_dry', cdry.body.m, 'meta', meta, ...
             'simin', simin, 'T', T, 'TSW', T.HOV1 - 0.5, 'DEST', G.(C.destVar));
end
