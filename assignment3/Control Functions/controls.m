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

function U_manoeurve = controls(U_trimmed, currentTime)

    % Set new vector
    U_manoeurve = U_trimmed;
     
%     % Change throttle
    if currentTime > 1 && currentTime < 28       
        U_manoeurve(1) = 1;
    end
    
    % Change elevator deflection
    if currentTime > 1 && currentTime < 28       
        U_manoeurve(2) = U_trimmed(2) - deg2rad(2.5);
    end
     
%     % Change aileron deflection
%     if currentTime > 1 && currentTime < 18       
%         U_manoeurve(3) = deg2rad(0);
%     end
    
%     % Change rudder deflection
%     if currentTime > 1 && currentTime < 18       
%         U_manoeurve(4) = deg2rad(0);
%     end
end