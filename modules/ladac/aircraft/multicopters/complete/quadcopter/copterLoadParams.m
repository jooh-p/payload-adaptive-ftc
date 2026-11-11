function copter = copterLoadParams( filename )
% copterLoadParams loads a quadcopter parameters struct.
% 
% Example:
%  copter = copterLoadParams( 'copter_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
%
% *** 우리 연구팀 해설 ***
% 기체 로더. filename(예 'copter_params_IRIS')을 실행해 copter 구조체 생성.
%   init에서 copter = copterLoadParams('copter_params_IRIS')로 호출.
%   ★ 우리 mkplant는 이렇게 만든 copter를 복사·수정(질량·관성)해 화물 주입.
% *************************************************************************

% 파라미터 .m 파일을 그대로 실행 → copter.body/config/prop... 필드가 정의됨
run(filename);

% initialize propeller map from propeller name
% 프로펠러 이름('10x4.7SF')으로 특성맵 fit 생성 → propellerOp가 추력·토크 계산에 사용
copter.prop.map_fit = propMapFitCreate( copter.prop.name, copter.prop.correction_factor );

end