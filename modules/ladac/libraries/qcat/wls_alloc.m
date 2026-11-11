function [u,W,iter] = wls_alloc(B,v,umin,umax,Wv,Wu,ud,gam,u,W,imax)
  
% WLS_ALLOC - Control allocation using weighted least squares.
%
%  [u,W,iter] = wls_alloc(B,v,umin,umax,[Wv,Wu,ud,gamma,u0,W0,imax])
%
% Solves the weighted, bounded least-squares problem
%
%   min ||Wu(u-ud)||^2 + gamma ||Wv(Bu-v)||^2
%
%   subj. to  umin <= u <= umax
%
% using an active set method.
%
%  Inputs:
%  -------
% B     control effectiveness matrix (k x m)
% v     commanded virtual control (k x 1)
% umin  lower position limits (m x 1)
% umax  upper position limits (m x 1)
% Wv    virtual control weighting matrix (k x k) [I]
% Wu    control weighting matrix (m x m) [I]
% ud    desired control (m x 1) [0]
% gamma weight (scalar) [1e6]
% u0    initial point (m x 1)
% W0    initial working set (m x 1) [empty]
% imax  max no. of iterations [100]
% 
%  Outputs:
%  -------
% u     optimal control
% W     optimal active set
% iter  no. of iterations (= no. of changes in the working set + 1)
%
%                            0 if u_i not saturated
% Working set syntax: W_i = -1 if u_i = umin_i
%                           +1 if u_i = umax_i
%
% See also: WLSC_ALLOC, IP_ALLOC, FXP_ALLOC, QP_SIM.
  
% *** 우리 연구팀 해설 ***
% ★ "요(yaw) 포기"가 수학적으로 실현되는 최종 지점 (QCAT, Härkegård).
%   푸는 문제: min ||Wu(u-ud)||^2 + gamma*||Wv(Bu-v)||^2  s.t. umin<=u<=umax
%     Wv = 축(롤·피치·요·추력) 가중치. 요=0.01로 작아 목적함수에서 요 오차가
%          거의 무시됨 → 모터 포화로 다 못 맞출 때 요부터 희생.
%     롤·피치(30)·추력(300)은 큰 가중이라 우선 유지.
%   방법 = active set: 포화 모터를 하나씩 고정하며 나머지로 최소자승 반복.
%   caIndiWls가 이 함수를 호출.

% Number of variables
  m = length(umin);
  
  % Set default values of optional arguments
  if nargin < 11
    imax = 100; % Heuristic value
    [k,m] = size(B);
    if nargin < 10, u = (umin+umax)/2; W = zeros(m,1); end
    if nargin < 8,  gam = 1e6;       end
    if nargin < 7,  ud = zeros(m,1); end
    if nargin < 6,  Wu = eye(m);     end
    if nargin < 5,  Wv = eye(k);     end
  end
      
  % gamma(1e6)와 Wv를 곱해 "원하는 힘 맞추기(Bu=v)"를 강하게 우선.
  % 단 Wv 가중이 축별로 달라 요(0.01) 항은 상대적으로 약함 → 요 희생의 근원.
  gam_sq = sqrt(gam);
  A = [gam_sq*Wv*B ; Wu];
  b = [gam_sq*Wv*v ; Wu*ud];
  
  % Initial residual.
  d = b - A*u;
  % Determine indeces of free variables.
  i_free = W==0;
  
  % Iterate until optimum is found or maximum number of iterations
  % is reached.
  for iter = 1:imax
    % ----------------------------------------
    %  Compute optimal perturbation vector p.
    % ----------------------------------------
    
    % Eliminate saturated variables.
    % [해설] 포화 안 된(자유) 모터만으로 축소 최소자승 → 이동방향 p_free
    A_free = A(:,i_free);
    % Solve the reduced optimization problem for free variables.
    p_free = A_free\d;
    % Zero all perturbations corresponding to active constraints.
    p = zeros(m,1);
    % Insert perturbations from p_free into free the variables.
    p(i_free) = p_free;
    
    % ----------------------------
    %  Is the new point feasible?
    % ----------------------------
    
    u_opt = u + p;
    infeasible = (u_opt < umin) | (u_opt > umax);

    if ~any(infeasible(i_free))

      % ----------------------------
      %  Yes, check for optimality.
      % ----------------------------
      
      % Update point and residual.
      u = u_opt;
      d = d - A_free*p_free;
      % [해설] 최적성 판정: 아래 Lagrange 승수가 모두 >=0이면 최적(포화 배치가 옳음).
      % Compute Lagrangian multipliers.
      lambda = W.*(A'*d);
      % Are all lambda non-negative?
      if lambda >= -eps
	% / ------------------------ \
	% | Optimum found, bail out. |
	% \ ------------------------ /
	return;
      end
      
      % --------------------------------------------------
      %  Optimum not found, remove one active constraint.
      % --------------------------------------------------
      
      % Remove constraint with most negative lambda from the
      % working set.
      [lambda_neg,i_neg] = min(lambda);
      W(i_neg) = 0;
      i_free(i_neg) = 1;
    
    else
      
      % ---------------------------------------
      %  No, find primary bounding constraint.
      % ---------------------------------------
      
      % [해설] 한계 넘으면: 경계까지만 이동(alpha) 후 그 모터를 포화(active)로 고정.
      %   포화 모터가 늘수록 남은 자유도가 줄어 요를 못 맞춤 → 요가 희생되는 실제 메커니즘.
      % Compute distances to the different boundaries. Since alpha < 1
      % is the maximum step length, initiate with ones.
      dist = ones(m,1);
      i_min = i_free & p<0;
      i_max = i_free & p>0;

      dist(i_min) = (umin(i_min) - u(i_min)) ./ p(i_min);
      dist(i_max) = (umax(i_max) - u(i_max)) ./ p(i_max);

      % Proportion of p to travel
      [alpha,i_alpha] = min(dist);
      % Update point and residual.
      u = u + alpha*p;
      d = d - A_free*alpha*p_free;
      
      % Add corresponding constraint to working set.
      W(i_alpha) = sign(p(i_alpha));
      i_free(i_alpha) = 0;
      
    end
  
  end
  