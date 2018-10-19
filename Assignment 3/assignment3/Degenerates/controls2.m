% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460398189
% Function Name: controls2 - Aileron input
%
% Function Description:
%   Determines the control input vector for a manoeuvre
%
% Inputs:
%   U:          Vector containing all aircraft control settings. The order is:
%                   - delta_t = U(1)    -
%                   - delta_e = U(2)    (rad)
%                   - delta_a = U(3)    (rad)
%                   - delta_r = U(4)    (rad)
%   Time:       Time simulation vector
%   CurrentTime:Current time of the simutlation 
%
% Outputs:
%   U_manoeuvre:  Aircraft state at end of time step. Same order and units as 
%           'X0', listed above
%
% Other m-files required:
%   None
%
% Subfunctions:
%   None
%
% MAT-files required: none
%
% TODO: 
%   NOTHING - CHECK IF CORRECT

function U_manoeuvre = controls2(Params, X, U, currentTime)

    % Quesiton 1, elevator deflection of 5 degrees
    A_deflection = deg2rad(0.1);
        
    % Set new vector
    U_manoeuvre = U;
    
    % Change elevator deflection for 0.5 seconds by 5 deg
    if currentTime > 10 && currentTime < 10.5 

        U_manoeuvre(3) = A_deflection;

    end 
        
end