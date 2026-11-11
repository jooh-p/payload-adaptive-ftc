function [eul_ang_des,T_spec_des,n_g_des] = indiCopterAcc2AttiSmeur( nu_s_g_dt2, s_g_dt2, M_bg, g )
% indiCopterAcc2AttiSmeur desired attitude and thrust for copters with INDI
%   This function can be used as INDI interface between a position
%   controller cascade and an attitude controller cascade for multicopters.
%   This function is implemented according to [1].
%   Note that the function indiCopterAcc2LeanVector should be preferable to
%   this function.
% 
% Syntax:
%   [eul_ang_des,T_spec_des,n_g_des] = indiCopterAcc2AttiSmeur( ...
%       nu_s_g_dt2, s_g_dt2, M_bg, g )
% 
% Inputs:
%   nu_s_g_dt2      pseudo-control input from position controller cascade;
%                   commanded 2nd time-derivative of the position in
%                   geodetic (g) frame (3x1 array), in m/s^2
%   s_g_dt2         measured 2nd time-derivative of the position in
%                   geodetic (g) frame (3x1 array), in m/s^2
%   M_bg            rotation matrix from geodetic (g) frame to body-fixed
%                   (b) frame (3x3 array), dimensionless
%   g               gravitational acceleration (scalar), in m/s^2
% 
% Outputs:
%   eul_ang_des     desired euler angles (3x1 array), in rad
%   T_spec_des      desired specific thrust (scalar), in m/s^2
%   n_g_des         desired lean vector as input for the attitude
%                   controller cascade in geodetic (g) frame; the lean
%                   vector is used for reduced attitude control, it points
%                   into the direction of the thrust vector and is a unit
%                   vector (3x1 array), see [1], section II.B.,
%                   dimensionless
% 
% Literature:
%   [1] Smeur, E. J., de Croon, G. C., & Chu, Q. (2018). Cascaded
%       incremental nonlinear dynamic inversion for MAV disturbance 
%       rejection. Control Engineering Practice, 73, 79-90.
% 
% See also:
% 	indiCopterAcc2LeanVector, ndiCopterAcc2LeanVector

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% *************************************************************************
%
% *** 우리 연구팀 해설 ***
% 위치제어 ↔ 자세제어 사이 INDI 인터페이스(캐스케이드 다리, 파이프라인 2→3단).
%   원하는 위치 가속도(nu_s_g_dt2) → 원하는 자세(euler)+추력으로 변환.
%   ★ INDI 증분: 원하는 가속도와 "측정" 가속도의 차이만 자코비안 G로 역변환
%     → 현재 자세에서 얼마 기울일지 증분 계산(모델오차·화물에 강인).
% *************************************************************************


% to do: avoid singularities
euler_angles = dcm2Euler(M_bg);
phi         = euler_angles(1);
theta       = euler_angles(2);
psi         = euler_angles(3);
sin_phi     = sin(phi);
cos_phi     = cos(phi);
sin_theta   = sin(theta);
cos_theta   = cos(theta);
sin_psi     = sin(psi);
cos_psi     = cos(psi);

g_g         = [0;0;g];
n_g_meas	= dcm2LeanVector( M_bg );   % 현재 기울기(추력 방향) 단위벡터
% thrust defined negative
% 현재 추력 = 측정 가속도(중력 제외)를 추력 방향으로 투영
T = -dot(n_g_meas,(s_g_dt2-g_g));

G = [ ...
    (cos_phi*sin_psi-sin_phi*cos_psi*sin_theta)*T, cos_phi*cos_psi*cos_theta*T, sin_phi*sin_psi+cos_phi*cos_psi*sin_theta; ...
    (-sin_phi*sin_psi*sin_theta-cos_psi*cos_phi)*T, cos_phi*sin_psi*cos_theta*T, cos_phi*sin_psi*sin_theta-cos_psi*sin_phi; ...
    -cos_theta*sin_phi*T, -sin_theta*cos_phi*T, cos_phi*cos_theta ...
    ];
u_f         = [phi;theta;T];   % 현재 [롤, 피치, 추력]
% ★ INDI 증분 역변환: 현재값 + G⁻¹·(원하는가속도 − 측정가속도)
%   = 가속도 오차를 자세·추력 증분으로. 측정값 사용이 INDI 강인성의 핵심.
u_c         = u_f + pinv(G)*(nu_s_g_dt2-s_g_dt2);
eul_ang_des = [u_c(1);u_c(2);psi];   % 원하는 자세(요는 유지)
T_spec_des  = -u_c(3);               % 원하는 추력
M_bg_c      = euler2Dcm(eul_ang_des);
n_g_des     = dcm2LeanVector( M_bg_c );

end