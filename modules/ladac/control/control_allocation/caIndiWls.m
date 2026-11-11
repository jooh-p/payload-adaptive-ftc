function [ Delta_u, W, iter ] = caIndiWls( ca, ...
    B, Delta_nu, u, Delta_u_d, Delta_gamma, Delta_diag_W_v )
% caIndiWls adapt WLS control allocation for INDI
%   The adaption of WLS control allocation for INDI is described in [1].
% 
% Inputs:
%   ca                  control allocation parameters struct, see
%                       controlAllocationWlsLoadParams
%   B                   control effectiveness matrix (NxM array)
%   Delta_nu            Incremental pseudo-control input vector (Nx1 array)
%   u                   current control input vector (Mx1 array)
%   Delta_u_d           online adjustment of desired u (scalar or Mx1
%                       array); should be 0 if not needed.
%   Delta_gamma         online adjustment of ca.gamma (scalar); should be 0
%                       if not needed.
%   Delta_diag_W_v      online adjustment of ca.W_v diagonal (scalar or 
%                       Nx1 array); should be 0 if not needed
%   
% Outputs:
%   Delta_u             optimal incremental control input vector (Mx1
%                       array)
%   W                   optimal active set (Mx1 array)
%   iter                number of iterations (scalar)
% 
% Literature:
%   [1] Smeur, E., Höppener, D., & Wagter, C. D. (2017). Prioritized
%       Control Allocation for Quadrotors Subject to Saturation, In H. D. 
%       P. J.-M. Moschetta, G. Hattenberger (Ed.), International Micro Air
%       Vehicle Conference and Flight Competition 2017 (pp. 37-43).
%       Toulouse, France.
% 
% See also:
%   wls_alloc

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
%
% ─────────────────────────────────────────────────────────────────────
% 우리 연구팀 해설:
%   INDI용 가중최소자승(WLS) 제어할당. "요(yaw) 포기"가 실제로 실행되는 곳.
%   원하는 증분 모멘트/추력(Delta_nu)을 모터 명령 증분(Delta_u)으로 분배.
%   모터 포화로 다 못 맞추면 → W_v 가중치 낮은 축(요=0.01)부터 희생.
%   즉 로터1개 고장으로 추력 여유 없을 때 요를 버리고 롤·피치·추력을 지킴.
%   실제 최소자승은 맨 아래 wls_alloc가 풂(Smeur 2017 우선순위 알고리즘).
% ─────────────────────────────────────────────────────────────────────

% adjustments according to [1]
% [증분 형태로 변환] 현재 명령 u 기준 상대값(INDI는 증분 제어).
umin    = ca.u_min - u;   % 하한까지 여유
umax    = ca.u_max - u;   % 상한(포화)까지 여유 ← 고장 시 여기 걸림
ud      = ca.u_d - u;     % 원하는 명령까지 차이
u0      = 0.5 * (umin+umax);

% adjustments of online WLS parameter changes
% ★ 축별 우선순위 가중치. W_v(요)=0.01이면 최소자승이 요 오차를 거의 무시.
%   온라인으로 Delta_diag_W_v 더해 실시간 조정 가능(우리는 미사용).
gamma   = ca.gamma + Delta_gamma;
ud      = ud + Delta_u_d;
W_v     = ca.W_v + diag( Delta_diag_W_v );

% 실제 WLS 풀이: 모터한계(umin~umax) 안에서 원하는 힘에 최대한 근접.
%   포화 시 우선순위 낮은 축(요)부터 희생 → 최적 증분 명령 Delta_u 반환.
[ Delta_u, W, iter ] = wls_alloc( B, Delta_nu, umin, umax, ...
    W_v, ca.W_u, ud, gamma, u0, ca.W, ca.i_max );

end
