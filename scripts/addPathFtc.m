function [] = addPathFtc()

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

this_file_path = fileparts(mfilename('fullpath'));
cd(this_file_path);
addpath(genpath(this_file_path));

% ★ _archive 는 경로에서 뺀다 (2026-09-27)
%   보관 스크립트는 옛 기체·옛 시나리오·옛 파라미터 표를 쓴다. 경로에 있으면
%   이름만으로 불려 조용히 옛 값으로 돌아간다. 빼 두면 "정의되지 않은 함수" 로
%   크게 실패한다.
%   되살릴 때는 그 자리에서 직접:
%       addpath(genpath(fullfile(fileparts(which('addPathFtc')), '_archive')))
%   되살리기 전에 확인할 것 셋 — `_archive/README.md`
arch_ = fullfile(this_file_path, '_archive');
if isfolder(arch_), rmpath(genpath(arch_)); end

cd ..

% add folders to path
addpath(genpath('modules'));
addpath(genpath('models'));
addpath(genpath('functions'));
addpath(genpath('data'));
addpath(genpath('animation'));   % animate_*.m (루트서 이동)
addpath(genpath('figures'));     % 저장된 .fig 그림

cd(this_file_path);

end