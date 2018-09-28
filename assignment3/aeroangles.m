% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: aeroangles
%
% Function Description:
% Converts body velocity vector into flight vector components 
%
% Inputs:
%   Input: - vector of body velocities in the form [u;v;w]
%
% Outputs:
%   Output: - total velocity magnitude (V)
%           - angle of attack (alpha)
%           - side slip angle (beta)

function [V, alpha, beta] = aeroangles(bodyVec)

    % Unpack body vector
    u(:) = bodyVec(1,:);
    v(:) = bodyVec(2,:);
    w(:) = bodyVec(3,:);

    % Calculate total velocity
    V(:) = sqrt(u.^2 + v.^2 + w.^2);

    % Calculate angle of attack
    alpha(:) = atan(w./u);

    % Calculate side slip angle
    beta(:) = asin(V./u);

end