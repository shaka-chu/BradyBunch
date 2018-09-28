% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: rotatez
%
% Function Description:
% Performs rotations about the z-axis through a specified angle
%
% Inputs:
%   Input: - angle of rotation (in radians)
%
% Outputs:
%   Output: - rotation matrix

function cz = rotatez(angle)
    
    % Create matrix
    cz = [ cos(angle),  sin(angle),  0;
          -sin(angle),  cos(angle),  0;
                    0,           0,  1];
       
end