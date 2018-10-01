% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: staterates
%
% Function Description:
% Computes the rates of changes of all state variables
%
% Inputs:
%   X: Column vector of state variables
%
% Outputs:
%   Xdot: Column vector of rates of changes of state variables

function [Xdot] = staterates(X, Params, Ft, F_body, M_body, Fgx, Fgy, Fgz)

    % Unpack state vector
    u = X(1,:);
    v = X(2,:);
    w = X(3,:);
    p = X(4,:);
    q = X(5,:);
    r = X(6,:);
    q0 = X(7,:);
    q1 = X(8,:);
    q2 = X(9,:);
    q3 = X(10,:);
    x = X(11,:);
    y = X(12,:);
    z = X(13,:);
    
    % Unpack body forces and moments
    Fa_x = F_body(1);
    Fa_y = F_body(2);
    Fa_z = F_body(3);
    La = M_body(1);
    Ma = M_body(2);
    Na = M_body(3);
    
    % Extract parameters
    g = Params.Inertial.g;         
    m = Params.Inertial.m;          
    Ixx = Params.Inertial.Ixx;         
    Iyy = Params.Inertial.Iyy;      
    Izz = Params.Inertial.Izz ;      
    Ixz = Params.Inertial.Ixz;
    
    % Calculate inertial constants
    c0 = Ixx*Izz - Ixz^2;
    c1 = Izz/c0;
    c2 = Ixz/c0;
    c3 = c2*(Ixx - Iyy + Izz);
    c4 = c1*(Iyy - Izz) - c2*Ixz;
    c5 = 1/Iyy;
    c6 = c5*Ixz;
    c7 = c5*(Izz - Ixx);
    c8 = Ixx/c0;
    c9 = c8*(Ixx - Iyy) + c2*Ixz;
    
    % Calculate body accelerations
    udot = r.*v - q.*w - Fgx + (Fa_x + Ft)./m;
    vdot = -r.*u + p.*w + Fgy + Fa_y./m;
    rdot = q.*u - p.*v + Fgz + Fa_z./m;
    
    % Calculate manoeuvring rates
    pdot = c3.*p.*q + c4.*q.*r + c1.*La + c2.*Na;
    qdot = c7.*p.*r - c6.*(p.^2 - r.^2) + c5.*Ma;
    rdot = c9.*p.*q - c3.*q.*r + c2.*La + c8.*Na;
    
    % Calculate quaternion rates
    q0dot = -0.5.*(q1.*p + q2.*q + q3.*r);
    q1dot = 0.5.*(q0.*p - q3.*q + q2.*r);
    q2dot = 0.5.*(q3.*p + q0.*q - q1.*r);
    q3dot = -0.5.*(q2.*p - q1.*q - q0.*r);
    
    % Calculate position rates
    Cbe = rotate321quat([q0;q1;q2;q3]);
    [xedot; yedot; zedot] = inv(Cbe)*[u,v,w];
    
    % Create output
    Xdot = [udot;vdot;wdot;pdot;qdot;rdot;q0dot;q1dot;q2dot;q3dot;xedot;yedot;zedot];
    
     
end