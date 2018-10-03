% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: gravity
%
% Function Description:
% Determines the force due to gravity and converts it to the body frame
%
% Inputs:
%   Input: - mass of aircraft (mass)
%          - attitude in quaternions (q0,q1,q2,q3)
%
% Outputs:
%   Output: - body force components (Fgx,Fgy,Fgz)

function [Fgx, Fgy, Fgz] = gravity(Params, X)

    % Extract parameters
    g = Params.Inertial.g;
    m = Params.Inertial.m;

    % Unpack state vector
    q0  = X(7);
    q1  = X(8);
    q2  = X(9);
    q3  = X(10);
    
    % Calculate weight force
    F = m*g;
    
    % Create transformation matrix from Earth to body
    Cbe = rotate321quat([q0;q1;q2;q3]);

    % Transform force
    Fg = Cbe*F;
    Fgx = Fg(1);
    Fgy = Fg(2);
    Fgz = Fg(3);
end