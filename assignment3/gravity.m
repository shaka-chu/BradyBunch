% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: gravity
%
% Function Description:
%   Determines the force due to gravity and converts it to the body frame
%
% Inputs:
%   Params: Struct containing all characteristics of the aircraft
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
%
% Outputs:
%   Fgx: Body x-component of weight (N)
%   Fgy: Body y-component of weight (N)
%   Fgz: Body z-component of weight (N)
% 
% Other m-files required:
%   rotate321quat.m
%
% Subfunctions:
%   rotate321quat
%
% MAT-files required: none
%
% TODO: none

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
    F = [0; 0; m*g];
    
    % Create transformation matrix from body to Earth
    Cbe = rotate321quat([q0;q1;q2;q3]);
    
    % Transformation matrix from Earth to body
    Ceb = Cbe';
    
%     quaternion = [q0;q1;q2;q3];
%     
%     euler_angles = quat2euler(quaternion);
%     
%     phi = euler_angles(1);
%     theta = euler_angles(2);
%     psi = euler_angles(3);
%     
%     Ceb = [cos(psi)*cos(theta), (cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi)), (cos(phi)*sin(theta)*cos(phi)+sin(psi)*sin(phi));
%            sin(psi)*cos(theta), (sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi)), (sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi));
%            -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
    

    % Transform force
    Fg = Ceb*F;
%     Fg = Cbe'*F;
    Fgx = Fg(1);
    Fgy = Fg(2);
    Fgz = Fg(3);
end