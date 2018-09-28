% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: rotatey
%
% Function Description:
% Performs rotations about the y-axis through a specified angle
%
% Inputs:
%   Input: - angle of rotation (in radians)
%
% Outputs:
%   Output: - rotation matrix

function cy = rotatey(angle)
    
    % Create matrix
    cy = [ cos(angle),  0, -sin(angle);
                    0,  1,           0;
           sin(angle),  0,  cos(angle)];
       
end