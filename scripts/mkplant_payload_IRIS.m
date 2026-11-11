function c = mkplant_payload_IRIS(cdry, pm, poff, usebox, dim)
%% mkplant_payload_IRIS — build the drone model with a payload
%
% Inputs
%   cdry    payload-free airframe struct (copterLoadParams('copter_params_IRIS'))
%   pm      payload mass [kg]
%   poff    mounting depth [m] — box centre below the airframe centre
%   usebox  true (default) = uniform-density box · false = point-mass approximation
%   dim     box size [length width height], default [0.15 0.15 0.10] m
%
% Physics
%   mass     M = m_dry + pm
%   inertia  I = I_dry
%              + mu*poff^2*diag([1 1 0])                          offset (parallel-axis) inertia
%              + (pm/12)*diag([b^2+h^2, a^2+h^2, a^2+b^2])        box shape inertia
%            reduced mass mu = m_dry*pm/M  (accounts for both bodies' distances from the
%            new centre of mass; using pm directly overestimates the inertia)
%
%   ★ The box shape term is the only term that increases the yaw inertia.
%     For 340 g the yaw inertia goes from 0.03100 to 0.03227.
%
%   ★ Aerodynamics and centre of gravity are not changed.
%     - Centre of gravity: the drone model does not use its vertical component.
%     - Aerodynamics: the reference area enters rotational damping as S^1.5; adding the
%       box side area would make that term dominate without a basis for its size.
%
% Examples
%   plant = mkplant_payload_IRIS(cdry, 0.34, 0.10);          % box (default)
%   plant = mkplant_payload_IRIS(cdry, 0.34, 0.10, false);   % point mass

if nargin < 4 || isempty(usebox), usebox = true;             end
if nargin < 5 || isempty(dim),    dim    = [0.15 0.15 0.10]; end

M  = cdry.body.m + pm;
mu = cdry.body.m * pm / M;

c = cdry;
c.body.m = M;

I = cdry.body.I + mu*poff^2*diag([1 1 0]);          % offset inertia — roll and pitch only
if usebox && pm > 0
    a = dim(1);  b = dim(2);  h = dim(3);
    I = I + (pm/12)*diag([b^2+h^2, a^2+h^2, a^2+b^2]);   % box shape — also increases yaw
end
c.body.I = I;
end
