% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460369684
% Function Name: windforces
%
% Function Description:
%   Determines non-dimensional coefficients for the aerodynamic forces
%   generated in the z and x directions (in the aerodynamic axes)
%
% Inputs:
%   Params:  Struct containing all characteristics of the aircraft
%   alpha:   Aircraft angle of attack (radians)
%   X:      Vector containing the aircraft state. The order is:
%               - u   = X(1)    (m/s)
%               - v   = X(2)    (m/s)
%               - w   = X(3)    (m/s)
%               - p   = X(4)    (rad/s)
%               - q   = X(5)    (rad/s)
%               - r   = X(6)    (rad/s)
%               - q0  = X(7)    -
%               - q1  = X(8)    -
%               - q2  = X(9)    -
%               - q3  = X(10)   -
%               - x   = X(11)   (m)
%               - y   = X(12)   (m)
%               - z   = X(13)   (m)
%   U:      Vector containing all aircraft control settings. The order is:
%               - delta_t = U(1)    -
%               - delta_e = U(2)    (rad)
%               - delta_a = U(3)    (rad)
%               - delta_r = U(4)    (rad)
%   V:      Total velocity magnitude (m/s)
%
% Outputs:
%   Cfa_z: Non-dimensional coefficient for z-direction aerodynamic force
%   Cfa_x: Non-dimensional coefficient for x-direction aerodynamic force
%   CL:    Lift coefficient
% 
% Other m-files required: none
% 
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

function [Cfa_z, Cfa_x, CL] = windforces(Params, alpha, X, U, V, angle_rates, L)

    % Extract necessary aircraft parameters
    CLo     = Params.Aero.CLo;          % Non-dimensional
    CLa     = Params.Aero.CLa;          % /rad
    CLq     = Params.Aero.CLq;          % Non-dimensional
    CLde    = Params.Aero.CLde;         % /rad
    CLad    = Params.Aero.CLad;         % /rad
    Cdo     = Params.Aero.Cdo;          % Non-dimensional
    k       = Params.Aero.k;            % Non-dimensional
    c       = Params.Geo.c;             % m
    
    % Unpack angle rates of change (rad/s)
    alpha_dot = angle_rates(1);

    % Unpack state vector
    q   = X(5);

    % Non-dimensionalise angular rates
    q_hat           = (q*c)/(2*V);
    alpha_dot_hat   = (alpha_dot*c)/(2*V);

    % Unpack control vector
    delta_e = U(2);
    
    % Lift coefficient 
    CL = - CLo - CLa*alpha - CLq*q_hat - CLde*delta_e - CLad*alpha_dot_hat;
    
    % Drag coefficient (simple drag model)
    Cd = Cdo + k*CL^2;
    
    % Aerodynamic force coefficients in x and z directions
%     Cfa_z = CL - Cd*alpha;
%     Cfa_x = CL*alpha - Cd; 
    Cfa_z = CL;
    Cfa_x = Cd;
end