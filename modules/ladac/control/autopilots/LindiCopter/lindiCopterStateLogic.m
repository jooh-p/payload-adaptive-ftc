function state = lindiCopterStateLogic(mode_number,is_traj_valid)
% lindiCopterStateLogic define logical signals depending on flight mode
%
% 우리 연구팀 해설:
%   비행 모드 번호에 따라 어떤 제어 서브시스템을 켜고 끌지 정하는 스위치판.
%   각 불린이 Simulink 제어기 블록 on/off. 우리 실험은 mode 0(loiter).
%   플래그 뜻: Psc=위치제어(수평), PosRm=위치 기준모델(목표궤적),
%   VertPsc=수직(고도)제어, Gdnce=유도(궤적추종), AttiCmd=수동 자세명령,
%   ManThr=수동 스로틀, AutoTune=자동튜닝.

% init
state.isPscEnabled      = true;
state.isPosRmEnabled	= true;
state.isVertPscEnabled	= true;
state.isGdnceEnabled	= true;
state.isAttiCmdEnabled  = true;
state.isManThrEnabled	= true;
state.isAutoTuneEnabled = true;


% set variables

switch mode_number
    case 0 % loiter  (★ 우리 실험 모드: 수평위치 자동유지, 수직·자세명령·수동 off)
        state.isPscEnabled      = true;
        state.isPosRmEnabled	= true;
        state.isVertPscEnabled	= false;
        state.isGdnceEnabled	= false;
        state.isAttiCmdEnabled  = false;
        state.isManThrEnabled	= false;
        state.isAutoTuneEnabled = false;
    case 1 % guided (유도/궤적 추종 — 트래젝토리 있으면 위치RM 끔)
        state.isPscEnabled      = true;
        if ~is_traj_valid
            state.isPosRmEnabled = true;
        else
            state.isPosRmEnabled = false;
        end
        state.isVertPscEnabled	= false;
        state.isGdnceEnabled	= true;
        state.isAttiCmdEnabled  = false;
        state.isManThrEnabled	= false;
        state.isAutoTuneEnabled = false;
    case 2 % stabilized (수동조종: 위치 off, 자세명령+수동스로틀 on)
        state.isPscEnabled      = false;
        state.isPosRmEnabled	= false;
        state.isVertPscEnabled	= false;
        state.isGdnceEnabled	= false;
        state.isAttiCmdEnabled  = true;
        state.isManThrEnabled	= true;
        state.isAutoTuneEnabled = false;
    case 3 % altitude hold (고도유지: 수직위치제어 on)
        state.isPscEnabled      = true;
        state.isPosRmEnabled	= true;
        state.isVertPscEnabled	= true;
        state.isGdnceEnabled	= false;
        state.isAttiCmdEnabled  = true;
        state.isManThrEnabled	= false;
        state.isAutoTuneEnabled = false;
    case 4 % autotune (자동튜닝 모드)
        state.isPscEnabled      = true;
        state.isPosRmEnabled	= true;
        state.isVertPscEnabled	= false;
        state.isGdnceEnabled	= false;
        state.isAttiCmdEnabled  = false;
        state.isManThrEnabled	= false;
        state.isAutoTuneEnabled = true;
end

end