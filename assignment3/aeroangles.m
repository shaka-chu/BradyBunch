% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
    % Author SID: 460306678
% Fu    nction Name: aeroangles
%   
% Function Description:
% Converts body velocity vector into flight vector components 
%
% Inputs:
%   Input: - vectors of body velocity components u,v,w
%
% Outputs:
%   Output: - total velocity magnitude (V)
%           - angle of attack (alpha)
%           - side slip angle (beta)

function [V, alpha, beta] = aeroangles(X)

    % Unpack state vector
    u   = X(1)
    v   = X(2)
    w   = X(3)

    % Calculate total velocity
    V = sqrt(u.^2 + v.^2 + w.^2)

    % Calculate angle of attack
    alpha = atan(w./u);

    % Calculate side slip angle
    beta = asin(v/V);

end