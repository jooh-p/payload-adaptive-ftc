function verify_release_IRIS()
%% verify_release_IRIS — run six representative cases and print the metrics
%
% ┌─ Check card ────────────────────────────────────────────────────────
% │ ① Cases    가 0 g · 나 340 g · 다 340 g · 다 100 g (M2) · 다 340 g (M3) · 무고장 340 g
% │ ② Settings cfg_current_IRIS as is — nothing is overridden
% │ ③ Output   metrics of each case to 12 significant digits, saved to data/verify_release_<release>.mat
% │ ④ Check    compare with the expected results in README
% │ ⑤ Also     prints MATLAB version · Java · installed toolboxes
% └─────────────────────────────────────────────────────────────────────
%
% Case names: 가 = baseline, no payload · 나 = baseline, payload · 다 = proposed, payload · 무고장 = no failure
%
% Usage:  matlab -batch "verify_release_IRIS"   (6 cases, about 3–4 min)

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
