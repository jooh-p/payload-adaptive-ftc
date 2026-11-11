function V = basevars_IRIS(cdry, m_dry, meta, PM, POFF, T, TEND, withfail)
%% basevars_IRIS — 시뮬 변수를 구조체로 모은다 (★ 병렬 안전 · 단일 출처)
%
% ┌─ 왜 구조체인가 ─────────────────────────────────────────────────────
% │ assignin('base', ...) 은 **parfor 안에서 안 먹는다.** 워커마다 별도
% │ 작업공간이라 주 세션의 base 를 못 본다. 그냥 for 를 parfor 로 바꾸면
% │ 조용히 엉뚱한 값으로 돈다 — 에러도 안 난다.
% │
% │ 그래서 변수를 구조체로 모아 SimulationInput.setVariable 로 넘긴다.
% │ 이러면 직렬·병렬 어느 쪽에서도 같게 동작한다.
% │ 실행은 simseg_IRIS 를 쓴다.
% └─────────────────────────────────────────────────────────────────────
%
% 입력
%   cdry     무화물 기체 구조체
%   m_dry    무화물 질량 [kg]
%   meta     abc_IRIS.mat 의 ABC.meta
%   PM       화물 질량 [kg]
%   POFF     장착 깊이 [m]
%   T        시나리오 함수의 두 번째 출력 (현행 `scn_delivery3_IRIS`)
%   TEND     시뮬 종료 시각 [s]
%   withfail true 면 모터 2번을 T.TFAIL 에 죽인다
%
% 출력
%   V        setVariable 로 넘길 변수 구조체
%            제어기(fm_loiter)·선제추력(ffpar)·감지스위치(dseg_stop_en)는
%            여기 없다 — 구간마다 바뀌므로 setctrl_IRIS 로 얹는다.
%
% ★ 규약은 template_experiment_IRIS 참조.

V.copter = mkplant_payload_IRIS(cdry, PM, POFF);        % 규약(1) 상자 관성
V.envir  = envirLoadParams('params_envir','envir',0);   % 지면 고도 0
yaw0 = 0;  if isfield(T,'YAW0'), yaw0 = T.YAW0; end      % 시나리오가 초기 요를 정한다(2026-10-03 선회 추가)
V.IC     = struct('omega_Kb',[0;0;0], 'q_bg',euler2Quat([0;0;yaw0]), ...
                  'V_Kb',[0;0;0], 's_Kg',[0;0;0], 'omega_mot',[1;1;1;1]*480);
V.grnd   = groundLoadParams('params_ground_default');
V.pos_ref = struct('lat',37.6117, 'lon',-122.37822, 'alt',T.ALT0);  % 규약(2) 지상
V.fg      = struct('remoteURL','127.0.0.1', 'remotePort',5502);
V.westpar = [m_dry; meta.sumsq_dry; T.EST(1); T.EST(2)];            % 규약(4)
V.simin   = scn_delivery3_IRIS(TEND);                               % 규약(3) 배송 3판
%   ★ 2026-09-23 정정 — 종전에는 옛 1판(`scn_delivery_IRIS`)을 넣었다.
%     현행 호출자 넷(run_case · symfinal · othercraft · switch_at_detect)이
%     모두 바로 뒤에서 V.simin 을 덮어써서 무해했으나, 덮어쓰기를 잊으면
%     조용히 옛 시나리오로 돈다. 기본값도 현행으로 맞춘다.

for j = 1:4                                                          % 규약(5)
    if withfail && j==2, ft = T.TFAIL; else, ft = 1000; end
    V.(sprintf('failure_time_mot_%d', j)) = ft;
end
end
