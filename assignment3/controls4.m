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

function U_manoeurve = controls4(Params, X, U_trimmed, currentTime, U_linear, t_linear )
    
    % Set load factor
    n = 3.5;
    
    % Loop through t of inputs
   if currentTime <= t_linear(end)
        
        % Determine current position
        for i = 1:length(t_linear)
            if currentTime < 1.05*t_linear(i) && currentTime > 0.95*t_linear(i)
                break
            end
        end
       
        % Set input vector
        U_manoeurve(1) = U_linear(1,i);
        U_manoeurve(2) = U_linear(2,i);
        U_manoeurve(3) = U_linear(3,i);
        U_manoeurve(4) = U_linear(4,i);
        return

    else
        % Set trim vector
        U_manoeurve = U_trimmed;
    end

     

     
     

end