% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: rotatex
%
% Function Description:
% Performs rotations about the x-axis through a specified angle
%
% Inputs:
%   Input: - angle of rotation (in radians)
%
% Outputs:
%   Output: - rotation matrix

function cx = rotatex(angle)
    
    % Create matrix
    cx = [ 1,           0,          0;
           0,  cos(angle), sin(angle);
           0, -sin(angle), cos(angle)];
       
end