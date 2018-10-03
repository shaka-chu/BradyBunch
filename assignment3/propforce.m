% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: propforce
%
% Function Description:
% Determines the thrust produced by the propellor in the body frame
%
% Inputs:
%   Input: - Maximum sea level power (PmaxSL)
%          - Throttle setting (sigma)
%          - Propellor efficiency (eta)
%          - Tangential velocity (u)
%          - Trim setting (deltat)
%
% Outputs:
%   Output: - Thrust 

function thrust = propforce(Params, X, U, rho)

    % Extract aircraft parameters
    PmaxSL = Params.Prop.P_max;
    eta = Params.Prop.eta;
    
    % Extract state
    u = X(1);
    
    % Extract control
    delta_t = U(1);

    % Calculate sigma
    rho_SL = 1.225;
    sigma = rho/rho_SL;

    % Calculate available thrust
    Pmax = PmaxSL*(1.324*sigma - 0.324);

    % Calculate thrust produced
    thrust = (Pmax*eta*delta_t)/u;
end