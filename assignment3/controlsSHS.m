% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: controls
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
%   FINISH THIS FUNCTION

function U_manoeurve = controlsSHS(Params, U_trimmed, CL, time, i)
    
    % Set new vector
    U_manoeurve = U_trimmed;
    
    % Extract aircraft aero parameters
    Clda = Params.Aero.Clda;
    Cldr = Params.Aero.Cldr;
    Cnda = Params.Aero.Cnda;
    Cndr = Params.Aero.Cndr;
    Cydr = Params.Aero.Cydr;
    Cyb = Params.Aero.Cyb;
    Clb = Params.Aero.Clb;
    Cnb = Params.Aero.Cnb;
    
    % Solve for the control for a sideslip of 5 degrees
    if time < 15
        beta = deg2rad(5);
    else
        beta = 0;
    end
    A = [CL 0 0
        0 Clda Cldr
        0 Cnda Cndr];
    y = [-Cyb
        -Clb
        -Cnb]*beta;
    x = A\y;
    
    % Control inputs required for a sideslip of 5 degrees
    U_manoeurve(3:end) = x(2:end);
end