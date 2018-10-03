% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460369684
% Function Name: rungeKutta4
%
% Function Description:
%   Performs numerical integration on the aircraft state at time t_k to
%   obtain the state at t_k+1. Employs a 4th order Runge-Kutta integration
%   scheme
%
% Inputs:
%   Params: Struct containing all characteristics of the aircraft
%   X0:     Vector containing the aircraft state at the beginning of the 
%           time step. The order is:
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
%   dt:     Timestep size (s)
%
% Outputs:
%   X_new:  Aircraft state at 
%
% Other m-files required:
%   staterates, calculateForces.m, aeroangles.m, flowproperties.m, 
%   gravity.m, windforces.m, bodyforces.m, gravForces.m, propforce.m
%
% Subfunctions:
%   staterates.m, calculateForces, aeroangles, flowproperties, gravity,
%   windforces, bodyforces, gravForces, propforce
%
% MAT-files required: none
%
% TODO: none

function [X_new] = rungeKutta4(Params,X0,U,dt)
    
    % State rate at start of step
    X_dot_1 = staterates(Params, X0, U);
    
    % Incriment 
    An = X_dot_1*dt;
    
    % First state at middle of step
    X_2 = X_0 + An/2;
    
    % First state rate at middle of step
    X_dot_2 = staterates(Params, X_2, U);
    
    % Improved incriment 
    Bn = X_dot_2*dt;
    
    % Second state at middle of step
    X_3 = X_0 + Bn/2;
    
    % Second state rate at middle of step
    X_dot_3 = staterates(Params, X_3, U);
    
    % Improved incriment
    Cn = X_dot_3*dt;
    
    % State at end of step
    X_4 = X_0 + Cn;
    
    % State rate at end of step
    X_dot_4 = staterates(Params, X_4, U);
    
    % Improved incriment
    Dn = X_dot_4*dt;
    
    % Evaluate new state
    X_new = X_0 + (1/6)*(An + 2*Bn + 2*Cn + Dn);
end