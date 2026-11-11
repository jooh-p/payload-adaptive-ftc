function V = applyctrl_IRIS(V, meta, fm, delta, stopen)
%% applyctrl_IRIS — 제어기와 선제 추력 파라미터를 시뮬 변수 구조체에 넣는다 (2026-09-22 신설)
%
% symfinal_IRIS 와 fig3_abc4_IRIS 에 `cvf` 라는 이름으로 복제돼 있던 것.
% 지속 시간과 마스크는 cfg_current_IRIS 가 단일 출처.
%
% 입력
%   V       basevars_IRIS 가 만든 시뮬 변수 구조체
%   meta    abc_IRIS.mat 의 meta (제어 주기 TS · 감지 임계 TH · 지속 표본 PERSIST · RMAX · RTC)
%   fm      제어기 (mkctrl_current_IRIS)
%   delta   선제 추력 크기 (0 이면 주입 없음)
%   stopen  1 이면 감지 순간 모델이 스스로 정지(구간 실행)

C  = cfg_current_IRIS();
ts = meta.TS;
V.fm_loiter = fm;
V.ffpar = [delta; meta.TH; max(1, round(meta.PERSIST/ts)); max(1, round(C.tau/ts)); ...
           meta.RMAX; meta.RTC; ts; C.mask(:)];
V.dseg_stop_en = stopen;
end
