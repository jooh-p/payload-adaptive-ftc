% rigidBodyKinetics computes the time derivative of the velocity and
% angular velocity.
%   The times derivatives are computed depending on the force an moment
%   vector represented in body-fixed frame, mass and inertia matrix of the
%   body, the gravitational acceleration, the DCM from earth frame to
%   body-fixed frame as well as the velocity and the angular velocity.
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   R_b             three dimensional force vector represented in
%                   body-fixed frame, in N
%   Q_b             three dimensional moment vector represented in
%                   body-fixed frame, in N.m
%   m               scalar mass of the body, in kg
%   I_b             inertia 3x3 matrix in body-fixed frame, in kg.m^2
%   g               scalar gravitational acceleration, in m/s^2
%   M_bg            rotation 3x3 matrix (DCM) from the earth frame (g) to 
%                   the body-fixed frame (g), in 1         
%   omega_Kb        three dimensional angular velocity vector of the rigid
%                   body relative to the earth represented in body-fixed
%                   frame, in rad/s
%   V_Kb            three dimensional velocity vector of the rigid body
%                   relative to the earth represented in body-fixed frame,
%                   in m/s
% 
% Outputs:
%   omega_Kb_dt     three dimensional time derivative of the angular
%                   velocity vector of the rigid body relative to the earth
%                   represented in body-fixed frame, in rad/s^2
%   V_Kb_dt         three dimensional time derivative of the velocity
%                   vector of the rigid body relative to the earth
%                   represented in body-fixed frame, in m/^2
% 
% See also: rigidBodyKinematicsQuat
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%
% *** 우리 연구팀 해설 ***
% ★ Plant(기체 동역학)의 심장 = 뉴턴-오일러 방정식. 힘·모멘트 → 가속도.
%   우리 화물 관성(I_b)·질량(m)이 실제로 작용하는 곳(제어기가 아니라 실제 기체).
%   passive FTC의 "스핀 안정"이 아래 자이로항 cross(ω, I·ω)에서 나옴.
% *************************************************************************

function [omega_Kb_dt, V_Kb_dt] = ...
    rigidBodyKinetics( R_b, Q_b, m, I_b, g, M_bg, omega_Kb, V_Kb )%#codegen

% compute the time derivative of the angular velocity vector according to
% [1, page 36 or 42]
% [해설] 오일러 회전방정식: 각가속도 = I⁻¹·(모멘트 − 자이로항).
%   ★ I_b가 크면(화물) 같은 모멘트 Q_b로도 각가속도↓ → 자세 반응 둔해짐.
%   자이로항 cross(ω, I·ω) = 자전하는 기체의 세차운동(스핀 안정의 근원).
%   비대각 I(편심)면 이 항이 롤-요 커플링 유발 → 편심 실험 발산 원인.
omega_Kb_dt = I_b \ (Q_b - cross( omega_Kb, I_b * omega_Kb ) );

% compute the time derivative of the velocity vector according to [1, page
% 41 or 42]
% [해설] 병진 뉴턴: 가속도 = 힘/질량 + 중력 − 코리올리항.
%   R_b=추력·항력 합력. M_bg*[0;0;g]=중력을 기체좌표로 변환. 화물로 m↑ 시 가속도↓.
V_Kb_dt = 1/m * R_b + M_bg * [0; 0; g] - cross( omega_Kb, V_Kb );