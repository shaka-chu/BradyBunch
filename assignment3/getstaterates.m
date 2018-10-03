% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460373315
% Function Name: getstaterates
%
%
% Function Description:
% Returns the state rate from the state vector
%
% Inputs:
%   
%
% Outputs:
%   Xdot : State rate vector

function Xdot = getstaterates(Params, X, U, phi_0)
    
    % Determine aero angles from aeroangles.m function
    [V, alpha, beta] = aeroangles(X(1), X(2), X(3));

    % Determine flow properties from flowproperties.m function
    [rho, Q] = flowproperties(-X(end), V);

    % Determine gravitational forces in the body fram from the
    % gravity.m function
    [Fgx, Fgy, Fgz] = gravity(Params, X(7), X(8), X(9), X(10));

    % Determine wind forces
    [Cfa_z, Cfa_x, CL] = windforces(Params, alpha, X(5), U(2));

    % Determine propulsive forces
    [thrust] = propforce(Params, rho, X(1), U(1));

    % Determine body forces
    [F_body, M_body] = bodyforces(Params, Cfa_x, Cfa_z, CL, ...
        Q, alpha, beta, 0, 0, U(1), U(2),...
        U(4), X(4), X(5), X(6), phi_0);

    % Determine the state rate vector
    [Xdot] = staterates(X, Params, thrust, F_body, M_body, ...
        Fgx, Fgy, Fgz);
end