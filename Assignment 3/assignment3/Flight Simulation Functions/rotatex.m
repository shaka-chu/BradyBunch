% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678, 460369684, 460373315, 460369189
% Function Name: rotatex
%
% Function Description:
%   Performs rotations about the x-axis through a specified angle
%
% Inputs:
%   angle: Angle of rotation (rad)
%
% Outputs:
%   cx: Rotation matrix
% 
% Other m-files required: none
% 
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

function cx = rotatex(angle)
    
    % Create matrix
    cx = [ 1,           0,          0;
           0,  cos(angle), sin(angle);
           0, -sin(angle), cos(angle)];     
end