function [G1,G2] = indiCePropVar( cep, G10, G20, u ) %#codegen
% indiCePropVar computes the matrices G1 and G2 (see [1]) from 
%   the control effectiveness of a trim point and compensates that the
%   thrust and torque are usually quadratic functions w.r.t. the control
%   input.
%   G1 and G2 can be used for incremental inversion according to eq. (19)
%   (neglecting G3).
% 
% Inputs:
%   cep             propeller control effectiveness parameters struct, see
%                   indiCeProp_params_default
%   G10             control effectiveness matrix, see indiCePropFix
%   G20             control effectiveness matrix, see indiCePropFix
% 
% Outputs:
%   G1              control effectiveness matrix according to [1]
%   G2              control effectiveness matrix to compensate the control
%                   input derivative according to [2]
% 
% Literature:
%   [1] Smeur, E. J., Chu, Q., & de Croon, G. C. (2016). Adaptive
%       incremental nonlinear dynamic inversion for attitude control of
%       micro air vehicles. Journal of Guidance, Control, and Dynamics,
%       39(3), 450-461.
%   [2] Beyer, Y., Guecker, F., Bremers, E., Steen, M., & Hecker, P.
%       (2022). Incremental Passive Fault-Tolerant Control for Quadrotors
%       With up to Three Successive Rotor Failures. In 6th CEAS Conference
%       on Guidance, Navigation and Control (EuroGNC) 2022.

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
%
% ─────────────────────────────────────────────────────────────────────
% 우리 연구팀 해설:
%   INDI의 제어효과 행렬 G1·G2를 "현재 동작점"에서 계산.
%   추력·토크는 모터명령의 2차함수(∝ω²)라 비선형 → 지금 명령 u에서의
%   기울기(민감도)로 국소 선형화. 이 G1이 caIndiWls의 B로 들어가
%   "모멘트 증분 → 모터 증분" 역변환에 쓰임.
%   ★ INDI 강인성의 핵심: 전체 모델이 아닌 "현재 상태의 기울기"만 쓰므로
%     질량·관성 오차(화물)에 상대적으로 둔감 → 우리 화물 시나리오에 유리.
% ─────────────────────────────────────────────────────────────────────

% 현재 명령 u에서의 모터 정적 회전수
omega = motorStaticSpeed( cep.kt, cep.ri, cep.vb, cep.d, u );

% 명령에 대한 회전수 민감도 dω/du (동작점 기울기)
omega_du = motorStaticSpeedDeriv( cep.kt, cep.ri, cep.vb, cep.d, u );

G_omega = diag( omega_du );

% G1: 추력·토크 ∝ ω² 미분 → 2ω·(dω/du). diag(omega)가 현재 동작점을 반영.
G1 = G10 * diag( omega ) * G_omega;

% G2: 모터 명령 변화율(u̇) 보상 항.
G2 = G20 * G_omega;

end