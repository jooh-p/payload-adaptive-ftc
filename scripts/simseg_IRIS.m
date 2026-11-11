function o = simseg_IRIS(m, t0, t1, op, V)
%% simseg_IRIS — 구간 하나를 돌린다 (★ 병렬 안전 · 단일 출처)
%
% 변수를 base 작업공간이 아니라 SimulationInput 에 실어 넘기므로 parfor
% 안에서도 그대로 동작한다. 변수 구조체는 basevars_IRIS 로 만들고
% 제어기·선제추력은 setctrl_IRIS 로 얹는다.
%
% 입력
%   m   모델 이름 · t0/t1 구간 [s] · op 시작 운전점([] 이면 처음부터)
%   V   변수 구조체
% 출력
%   o   시뮬 출력 (다음 구간에 넘길 운전점은 o.opPt)

in = Simulink.SimulationInput(m);
in = in.setModelParameter('StartTime', num2str(t0), 'StopTime', num2str(t1));
in = in.setModelParameter('SaveFinalState','on', 'SaveOperatingPoint','on', ...
                          'FinalStateName','opPt');
fn = setdiff(fieldnames(V), {'blkpar'}, 'stable');
for i = 1:numel(fn), in = in.setVariable(fn{i}, V.(fn{i})); end
% ★ 블록 값 덮어쓰기 (2026-10-28) — V.blkpar = {경로(모델 이름 뒤), 파라미터, 값; ...}. 모델 파일은 바뀌지 않는다.
if isfield(V, 'blkpar')
    for i = 1:size(V.blkpar,1), in = in.setBlockParameter([m '/' V.blkpar{i,1}], V.blkpar{i,2}, V.blkpar{i,3}); end
end
if ~isempty(op), in = in.setInitialState(op); end
evalc('o = sim(in);');
end
