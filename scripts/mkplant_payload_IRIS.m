function c = mkplant_payload_IRIS(cdry, pm, poff, usebox, dim)
%% mkplant_payload_IRIS — 화물을 실은 드론 모델 생성
%
% 입력
%   cdry    무화물 기체 구조체 (copterLoadParams('copter_params_IRIS'))
%   pm      화물 질량 [kg]
%   poff    장착 깊이 [m] — 상자 중심이 기체 중심에서 아래로
%   usebox  true(기본) = 균일 밀도 상자 · false = 점질량 근사
%   dim     상자 치수 [앞뒤 좌우 높이], 기본 [0.15 0.15 0.10] m
%
% 물리
%   질량   M = m_dry + pm
%   관성   I = I_무화물
%            + mu*poff^2*diag([1 1 0])                          위치 관성
%            + (pm/12)*diag([b^2+h^2, a^2+h^2, a^2+b^2])        상자 형상 관성
%          환산질량 mu = m_dry*pm/M  (화물과 기체가 새 질량중심에서 떨어진
%          거리를 함께 담는다. pm 을 그대로 쓰면 관성이 과대평가된다.)
%
%   ★ 상자 형상 항은 요 관성을 늘리는 유일한 항이다. 340 g 기준 요 관성 0.03100 → 0.03227.
%
%   ★ 공력과 무게중심은 손대지 않는다.
%     - 무게중심: 드론 모델이 세로 성분을 쓰지 않는다.
%     - 공력: 기준면적이 회전 감쇠에 S^1.5 로 들어가는데, 상자 옆면적을
%       더하면 효과가 사실상 전부 그쪽에서 나오고 그 크기에 근거가 없다.
%
% 사용 예
%   plant = mkplant_payload_IRIS(cdry, 0.34, 0.10);          % 상자 (기본)
%   plant = mkplant_payload_IRIS(cdry, 0.34, 0.10, false);   % 점질량

if nargin < 4 || isempty(usebox), usebox = true;             end
if nargin < 5 || isempty(dim),    dim    = [0.15 0.15 0.10]; end

M  = cdry.body.m + pm;
mu = cdry.body.m * pm / M;

c = cdry;
c.body.m = M;

I = cdry.body.I + mu*poff^2*diag([1 1 0]);          % 위치 관성 — 롤·피치만
if usebox && pm > 0
    a = dim(1);  b = dim(2);  h = dim(3);
    I = I + (pm/12)*diag([b^2+h^2, a^2+h^2, a^2+b^2]);   % 상자 형상 — 요도 늘어난다
end
c.body.I = I;
end
