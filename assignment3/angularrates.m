% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: angularrates
%
% Function Description:
% Determines rates of change of angle of attack and side slip
%
% Inputs:
%   Input: - components of the flight vector (V,alpha,beta)
%          - rates of change of body velocity vector (vdot,wdot)
%
% Outputs:
%   Output: - rate of change of angle of attack (alphaDot)
%           - rate of change of side slip angle (betaDot)


function [alphaDot, betaDot] = angularrates(V,alpha,beta,vDot,wDot)

    % Calculate rate of change of angle of attack
    alphaDot = (wDot./V).*sec(alpha).*sec(beta);

    % Calculate rate of change of side slip angle
    betaDot = (vDot./V).*sec(beta);

end