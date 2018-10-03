% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460369684
% Function Name: windforces
%
% [CFA_Z, CFA_X] = WINDFORCES(PARAMS, ALPHA, Q_HAT, DELTA_E)
%
% Function Description:
% Determines non-dimensional coefficients for the aerodynamic forces
% generated in the z and x directions (in the aerodynamic axes)
%
% Inputs:
%   Params:  Struct containing all aircraft parameters for a given CG case
%   alpha:   Aircraft angle of attack (radians)
%   q_hat:   Pitch rate (non-dimensional)
%   delta_e: Elevator position (radians)
%
% Outputs:
%   Cfa_z: Non-dimensional coefficient for z-direction aerodynamic force
%   Cfa_x: Non-dimensional coefficient for x-direction aerodynamic force
%   CL:    Lift coefficient

function [Cfa_z, Cfa_x, CL] = windforces(Params, alpha, X, U, V)

    % Extract necessary aircraft parameters
    CLo     = Params.Aero.CLo;          % Non-dimensional
    CLa     = Params.Aero.CLa;          % /rad
    CLq     = Params.Aero.CLq;          % Non-dimensional
    CLde    = Params.Aero.CLde;         % /rad
    Cdo     = Params.Aero.Cdo;          % Non-dimensional
    k       = Params.Aero.k;            % Non-dimensional
    c       = Params.Geo.c;             % m

    % Unpack state vector
    q   = X(5);

    % Non-dimensionalise angular rate
    q_hat = (q*c)/(2*V);

    % Unpack control vector
    delta_e = U(2);
    
    % Lift coefficient 
    CL = -CLo - CLa*alpha - CLq*q_hat - CLde*delta_e;
    
    % Drag coefficient (simple drag model)
    Cd = Cdo + k*CL^2;
    
    % Aerodynamic force coefficients in x and z directions
    Cfa_z = CL - Cd*alpha;
    Cfa_x = CL*alpha - Cd; 
end