% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: aeroangles
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

function [V, alpha, beta] = aeroangles(u,v,w)

    % Unpack body vector
    u(:) = u;
    v(:) = v;
    w(:) = w;

    % Calculate total velocity
    V(:) = sqrt(u.^2 + v.^2 + w.^2);

    % Calculate angle of attack
    alpha(:) = atan(w./u);

    % Calculate side slip angle
    beta(:) = asin(V./u);

end