function verify_release_IRIS()
%% verify_release_IRIS — MATLAB 버전을 바꿔도 현행 결과가 같은가 (2026-10-28)
%
% ┌─ 왜 ────────────────────────────────────────────────────────────────
% │ R2026b 를 R2026a 옆에 설치했다. 원고와 표의 수치는 R2026a 에서 잰 것이다.
% │ 같은 판을 두 버전에서 돌려 지표가 같은지 본다.
% └─────────────────────────────────────────────────────────────────────
% ┌─ 검증 카드 ─────────────────────────────────────────────────────────
% │ ① 판      가 0 g · 나 340 g · 다 340 g · 다 100 g (M2) · 다 340 g (M3) · 무고장 340 g
% │ ② 조건    현행 그대로(cfg_current_IRIS) — 아무것도 덮어쓰지 않는다
% │ ③ 출력    각 판의 지표를 유효숫자 12자리로 찍고 data/verify_release_<릴리스>.mat 에 저장
% │ ④ 판정    두 버전의 출력을 밖에서 맞댄다 — 모든 값 차이 0 이면 동일
% │ ⑤ 함께    버전 · Java · 설치 툴박스 목록을 찍는다
% └─────────────────────────────────────────────────────────────────────
%
% 사용법:  matlab -batch "verify_release_IRIS"   (판 6, 약 3~4 분)

addPathFtc();
rel = version('-release');
fprintf('== 릴리스 R%s · %s\n', rel, version);
try, fprintf('== Java  %s\n', version('-java')); catch e, fprintf('== Java  없음 (%s)\n', e.message); end
v = ver;
fprintf('== 설치 제품 %d 종\n', numel(v));
for k = 1:numel(v), fprintf('   %s %s\n', v(k).Name, v(k).Version); end

ctx = ctx_current_IRIS();
CASES = {'가',0,2; '나',0.340,2; '다',0.340,2; '다',0.100,2; '다',0.340,3; '무고장',0.340,2};
R = cell(size(CASES,1),1);
for k = 1:size(CASES,1)
    t0 = tic;
    r = run_case_IRIS(ctx, CASES{k,1}, CASES{k,2}, CASES{k,3});
    R{k} = r;
    fprintf('판 %d  %s %3.0f g M%d  (%.1f 초)\n', k, CASES{k,1}, 1000*CASES{k,2}, CASES{k,3}, toc(t0));
    fprintf('   e %.12g | l %.12g | tilt %.12g | reH %.12g | alive %d\n', r.e, r.l, r.tilt, r.reH, r.alive);
    fprintf('   mest %.12g | fsel %.12g | dsel %.12g | tdet %.12g\n', r.mest, r.fsel, r.dsel, r.tdet);
    if strcmp(CASES{k,1}, '무고장')
        fprintf('   crdev %.12g | crspd %.12g | tiltmax %.12g\n', r.crdev, r.crspd, r.tiltmax);
    end
end
out = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', sprintf('verify_release_R%s.mat', rel));
save(out, 'CASES', 'R', 'rel');
fprintf('== 저장 %s\n== 끝\n', out);
end
