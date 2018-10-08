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

function U_manoeurve = controls5(Params, U_trimmed, V, currentTime)

    % Set new vector
    U_manoeurve = U_trimmed;
    
    % Estimate steady turn controls
    U_turn = steadyTurnEstimate(Params, U_trimmed, V);
     
    % Change throttle
    if currentTime > 1      
        U_manoeurve(1) = U_turn(1);
    end
    
    % Change elevator deflection
    if currentTime > 1      
        U_manoeurve(2) = U_turn(2) - deg2rad(0.3);
    end
     
    % Change aileron deflection
    if currentTime > 1
        U_manoeurve(3) = U_turn(3);
    end
    
    % Change rudder deflection
    if currentTime > 1   
        U_manoeurve(4) = U_turn(4);
    end
end