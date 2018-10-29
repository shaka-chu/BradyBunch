% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678, 460369684, 460373315, 460369189
% Function Name: rotatez
%
% Function Description:
%   Performs rotations about the z-axis through a specified angle
%
% Inputs:
%   angle: Angle of rotation (in rad)
%
% Outputs:
%   cz: Rotation matrix
% 
% Other m-files required: none
% 
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

function cz = rotatez(angle)
    
    % Create matrix
    cz = [ cos(angle),  sin(angle),  0;
          -sin(angle),  cos(angle),  0;
                    0,           0,  1];
       
end