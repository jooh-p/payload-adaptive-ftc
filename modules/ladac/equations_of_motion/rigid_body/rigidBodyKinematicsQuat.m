% rigidBodyKinematicsQuat computes the time derivative of quaternions and
% position.
%   The times derivatives are computed depending on the rotational
%   velocity, the velocity, the quaternion of the rigid body as well as the
%   DCM of the velocity frame to the position frame.
% 
% Literature:
%   [1] Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
%   [2] The MathWorks, Inc. (2002): Aerospace Blockset. For Use with 
%       Simulink. User's Guide. Version 1.
% 
% Inputs:
%   omega_Kb        three dimensional angular velocity vector of the rigid
%                   body relative to the earth represented in body-fixed
%                   frame, in rad/s
%   V_Kb            three dimensional velocity vector of the rigid body
%                   relative to the earth represented in body-fixed frame,
%                   in m/s
%   q_bg            four dimensional quaternion vector of the rigid body
%                   relativ to the earth, in 1
%   M_bg            rotation 3x3 matrix (DCM) from the earth frame (g) to 
%                   the body-fixed frame (g), in 1   
% 
% Outputs:
%   q_bg_dt         four dimensional vector of the quaternion time
%                   derivative, in 1/s
%   s_g_dt          three dimensional time derivative of the position 
%                   vector in earth frame, in m/s
% 
% See also: rigidBodyKinetics
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
%
% *** 우리 연구팀 해설 ***
% Plant 운동학 = kinetics(힘→가속도)의 짝. 속도·각속도 → 위치·자세로 적분.
%   ★ 우리가 분석하는 출력 s_g(위치)·Euler_angles(자세)가 여기서 나옴.
% *************************************************************************

function [ q_bg_dt, s_g_dt ] = ...
    rigidBodyKinematicsQuat( omega_Kb, V_Kb, q_bg, M_bg )%#codegen

% compute the omega cross product matrix according to [1, page 8]
omega_cross_product = [ 0, omega_Kb(3), -omega_Kb(2); ...
                        -omega_Kb(3), 0, omega_Kb(1); ...
                        omega_Kb(2), -omega_Kb(1), 0 ];

% normalize quaternion
q_bg_norm = quatNormalize( q_bg );

% [해설] 각속도 → 쿼터니언 변화율(자세 적분). 자세를 쿼터니언으로 갱신.
% compute the time derivative of the quaternions vector according to
% [1, page 51]
q_bg_dt = 0.5 * [ 0, -omega_Kb';...
                   omega_Kb, omega_cross_product ] ...
            * q_bg_norm;

% high gain quaternion normalization according to [2, page 3-60f]
k_quat = 1;
q_error = 1 - dot( q_bg, q_bg ); 
q_bg_dt = q_bg_dt + q_error * k_quat * q_bg_norm;

% [해설] 기체좌표 속도를 지구좌표로 변환 → 위치 변화율(위치 적분).
% computate of the time derivative of the position vector in g frame
% according to [1, page 41 or 42]
s_g_dt = M_bg' * V_Kb;