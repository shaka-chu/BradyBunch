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

function U_manoeurve = controls(U_trimmed, currentTime, U_filter, ...
    T_filter)

    % Initialise the input vector
    U_manoeurve = U_trimmed;

    % Loop through t of inputs
    if currentTime <= T_filter(end)

        % Determine current position
        for i = 1:length(T_filter)
            if currentTime < 1.05*T_filter(i) && currentTime > 0.95*T_filter(i)
                break
            end
        end
        
        % Check which inputs have been altered from the trim function
        updateBool = (any(U_filter(1:4, :), 2) ~= 0);
        
        % Change the length of the input
        U_filter = U_filter(1:4, i);

        % Set input vector
        U_manoeurve(updateBool) = U_filter(updateBool);
        return
    else
        % Set trim vector
        U_manoeurve = U_trimmed;
    end
end