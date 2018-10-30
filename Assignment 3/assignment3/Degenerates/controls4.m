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


function U_manoeurve = controls4(Params, X, U_trimmed, currentTime, U_filter, T_filter )

    
    % Set load factor
    n = 3.5;
    
    % Loop through t of inputs
   if currentTime <= T_filter(end)
        
        % Determine current position
        for i = 1:length(T_filter)
            if currentTime < 1.05*T_filter(i) && currentTime > 0.95*T_filter(i)
                break
            end
        end
       
        % Set input vector
        U_manoeurve(1) = U_filter(1,i);
        U_manoeurve(2) = U_filter(2,i);
        U_manoeurve(3) = U_filter(3,i);
        U_manoeurve(4) = U_filter(4,i);
        return

    else
        % Set trim vector
        U_manoeurve = U_trimmed;
   end


    % Check limits
    if U_manoeurve(1) <= Params.ControlLimits.Lower(1)
        disp('Control Limit Reached (1)')
    elseif U_manoeurve(2) <= Params.ControlLimits.Lower(2)
        disp('Control Limit Reached (2)')
    elseif U_manoeurve(3) <= Params.ControlLimits.Lower(3)
        disp('Control Limit Reached (3)')
    elseif U_manoeurve(4) <= Params.ControlLimits.Lower(4)
        disp('Control Limit Reached (4)')
    end
        

end