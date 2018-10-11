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

function U_manoeurve = controls(varargin)

    % Split up the variable input vector
    U_trimmed = varargin{1};
    currentTime = varargin{2};
    U_filter = varargin{3};
    T_filter = varargin{4};
    
    % The extra input to change from default
    if length(varargin) > 4
        
        % Boolean to ignore user's inputs of zeros, i.e. uses the
        % user's inputs of zeros in the simulation
        ignore_zero = varargin{5};
    else
        
        % Default for the ignore_zero boolean
        ignore_zero = true;
    end
    
    
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
        
        if ignore_zero
            
            % Check which inputs have been altered from the trim
            % function
            updateBool = (any(U_filter(1:4, :), 2) ~= 0);
            
        else
            
            % Check which inputs have been altered from the trim
            % function but use the trimmed input for user's inputs of
            % zeros
            updateBool = (U_filter(1:4, i) ~= 0);
        end
        
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