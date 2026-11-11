%  ** Parameter file of quadcopter IRIS (ArduPilot) **
% 
% Main sources: 
%   https://apps.dtic.mil/dtic/tr/fulltext/u2/1008930.pdf
%   http://www.arducopter.co.uk/iris-quadcopter-uav.html

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% body parameters
% 본체 질량·관성 — ★ 우리 mkplant/mkbox가 이 값을 복사해 화물을 더함.
% mass of vehicle, in kg
copter.body.m = 1.37;                     % 질량 1.37 kg
% inertial matrix of vehicle, in kg.m^2
% 관성행렬 diag[Ixx Iyy Izz]. 비대각=0 → 대칭기체(대칭화물 가정과 일치).
copter.body.I = [0.022, 0, 0; ...         % Ixx (롤)
            0, 0.011, 0; ...              % Iyy (피치)
            0, 0, 0.031];                 % Izz (요)

%% configuration parameters
% center of gravity position in c frame, in m
copter.config.CoG_Pos_c = [ 0; 0; 0 ];
% position of all propellers in c frame, in m
% 프로펠러 위치(각 행=1모터, 전치로 열=모터). x=앞뒤±0.13, y=좌우 앞0.22/뒤0.20
% → 좌우 폭이 앞뒤로 비대칭(우리 3D 애니가 이 형상 반영).
copter.config.propPos_c = [   0.13, -0.22, 0; ...   % M1 앞
                            0.13, 0.22, 0; ...       % M2 앞
                            -0.13, 0.2, 0; ...       % M3 뒤
                            -0.13, -0.2, 0 ]';       % M4 뒤
% direction of all propeller rotations, -1: right, 1 left
% 회전방향: 인접 모터 반대(CW/CCW 교대) → 반작용 토크 상쇄.
copter.config.propDir = [ 1; -1; 1; -1 ]*-1;
% orientation of the motors relative to the body
copter.config.M_b_prop1 = euler2Dcm([0,-pi/2,0]);
copter.config.M_b_prop2 = euler2Dcm([0,-pi/2,0]);
copter.config.M_b_prop3 = euler2Dcm([0,-pi/2,0]);
copter.config.M_b_prop4 = euler2Dcm([0,-pi/2,0]);
% hit points for ground model
copter.config.hitPoints_c = [ copter.config.propPos_c + [ 0; 0; 0.06 ], ...
                           copter.config.propPos_c + [ 0; 0; -0.03 ] ];

%% propeller parameters
% propeller moment of inertia, kg.m^2
copter.prop.I = 1/12*0.012*0.25^2 + 1/2*0.06*0.3*0.028^2;
% copter.prop.I = copter.prop.I * 9;
% propeller name spcifying propeller map (name must be inside database)
copter.prop.name = '10x4.7SF';
% factor to adjust thrust and torque (e.g. due to mounting)
copter.prop.correction_factor = 1;

%% motor parameters
% 모터 전기 파라미터 — 추력 한계·포화 계산의 근거(우리 추력 한계 실험과 직결).
% torque constant of the motor (KT=60/(2*pi*KV)), N.m/A
copter.motor.KT = 60/(2*pi*850);    % 토크상수 (KV=850 rpm/V)
% motor internal resistance, Ohm
copter.motor.R = 0.13;              % 내부저항

%% battery parameters
% battery voltage, V
copter.bat.V = 11.1;               % 배터리 전압 11.1V (3S) → 최대 회전수 상한 결정

%% aerodynamics
% reference surface of multicopter, in m^2
copter.aero.S = 0.06;
% minimum drag coefficient, in 1
copter.aero.C_Dmin = 0.2;
% maximum drag coefficient, in 1
copter.aero.C_Dmax = 0.5;
% maximum lift coefficient (whole vehicle), in 1
copter.aero.C_Lmax = 0.05;
% center of pressure (x_b, y_b direction), in m
copter.aero.CoP_xy = 0.05;
% center of pressure (z_b direction), in m
copter.aero.CoP_z = 0;
% damping moment coefficient for rotation, in N/(m^2*rad/s)
copter.aero.rate_damp = 0.47;