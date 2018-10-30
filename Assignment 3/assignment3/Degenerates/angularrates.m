% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: angularrates
%
% Function Description:
% Determines rates of change of angle of attack and side slip
%
% Inputs:
%   V:      total velocity magnitude (m/s)
%   alpha:  angle of attack (rad)
%   beta:   side slip angle (rad)
%   vDot:   Rate of change of body y-velocity
%   wDot:   Rate of change of body z-velocity
%
% Outputs:
%   alphaDot:   Rate of change of angle of attack (rad/s)
%   betaDot:    Rate of change of side slip angle (rad/s)
% 
% Other m-files required: none
% 
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

function [alphaDot, betaDot] = angularrates(V,alpha,beta,vDot,wDot)

    % Calculate rate of change of angle of attack
    alphaDot = (wDot./V).*sec(alpha).*sec(beta);

    % Calculate rate of change of side slip angle
    betaDot = (vDot./V).*sec(beta);
end