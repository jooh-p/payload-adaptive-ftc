function V = basevars_IRIS(cdry, m_dry, meta, PM, POFF, T, TEND, withfail)
%% basevars_IRIS — collect the simulation variables into a struct
%
% Variables are collected in a struct and passed with SimulationInput.setVariable
% instead of the base workspace, so the same code works in both for and parfor
% (parfor workers cannot see the base workspace of the main session). Run with simseg_IRIS.
%
% Inputs
%   cdry     payload-free airframe struct
%   m_dry    payload-free mass [kg]
%   meta     ABC.meta of abc_IRIS.mat
%   PM       payload mass [kg]
%   POFF     mounting depth [m]
%   T        second output of the scenario function (scn_delivery3_IRIS)
%   TEND     simulation end time [s]
%   withfail true = motor 2 stops at T.TFAIL
%
% Output
%   V        variable struct for setVariable
%            The controller (fm_loiter), feed-forward thrust (ffpar) and detection stop
%            switch (dseg_stop_en) change per segment and are added by applyctrl_IRIS.

V.copter = mkplant_payload_IRIS(cdry, PM, POFF);        % includes payload box inertia
V.envir  = envirLoadParams('params_envir','envir',0);   % ground altitude 0
yaw0 = 0;  if isfield(T,'YAW0'), yaw0 = T.YAW0; end      % initial yaw is set by the scenario
V.IC     = struct('omega_Kb',[0;0;0], 'q_bg',euler2Quat([0;0;yaw0]), ...
                  'V_Kb',[0;0;0], 's_Kg',[0;0;0], 'omega_mot',[1;1;1;1]*480);
V.grnd   = groundLoadParams('params_ground_default');
V.pos_ref = struct('lat',37.6117, 'lon',-122.37822, 'alt',T.ALT0);  % start on the ground
V.fg      = struct('remoteURL','127.0.0.1', 'remotePort',5502);
V.westpar = [m_dry; meta.sumsq_dry; T.EST(1); T.EST(2)];            % mass estimation — dry mass · reference sum of squared speeds · estimation window
V.simin   = scn_delivery3_IRIS(TEND);                               % delivery scenario

for j = 1:4                                                          % failure time (1000 s for healthy motors)
    if withfail && j==2, ft = T.TFAIL; else, ft = 1000; end
    V.(sprintf('failure_time_mot_%d', j)) = ft;
end
end
