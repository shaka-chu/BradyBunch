% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460398189, 460369684
% Function Name: calculateForces
%
% Function Description:
%   Returns all forces acting on the aircraft given its physical
%   characteristics, present state and control inputs
%
% Inputs:
%   Params: Struct containing all characteristics of the aircraft
%   X:      Vector containing the aircraft state. The order is:
%               - u   = X(1)    (m/s)
%               - v   = X(2)    (m/s)
%               - w   = X(3)    (m/s)
%               - p   = X(4)    (rad/s)
%               - q   = X(5)    (rad/s)
%               - r   = X(6)    (rad/s)
%               - q0  = X(7)    -
%               - q1  = X(8)    -
%               - q2  = X(9)    -
%               - q3  = X(10)   -
%               - x   = X(11)   (m)
%               - y   = X(12)   (m)
%               - z   = X(13)   (m)
%   U:      Vector containing all aircraft control settings. The order is:
%               - delta_t = U(1)    -
%               - delta_e = U(2)    (rad)
%               - delta_a = U(3)    (rad)
%               - delta_r = U(4)    (rad)
%
% Outputs:
%   Xdot: State rate vector with after iteration of alpha and beta dot
% 
% Other m-files required: none
%
% Subfunctions:
%   staterates, calculateForces, aeroangles, flowproperties, gravity, 
%   windforces, bodyforces, gravForces, propforce
%
% MAT-files required:
%   staterates.m, calculateForces.m, aeroangles.m, flowproperties.m, 
%   gravity.m, windforces.m, bodyforces.m, gravForces.m, propforce.m
%
% TODO:
%   CHECK THE HEADER

function [Xdot, CL, Y] = getstaterates(Params, X, U)
  
    % Initial guess
    alpha_dot_old = 0;
    beta_dot_old = 0;

    % Errors
    error = 1;
    tolerance = 1e-9;
    
    % Set iteration limit
    iterLim = 100;
    iterCount = 0;
    
    % Iterate until angle of attack and sideslip rates converge
    while error > tolerance
        
        % Concatenate previous iteration angle of attack/sideslip rates to
        % be passed to 'staterates' function
        angle_rates = [alpha_dot_old, beta_dot_old];

        % Estimate state rates using angle of attack and sideslip rates
        [Xdot, CL, Y] = staterates(Params, X, U, angle_rates);
        
        % Calculate angular rates
        [alpha_dot, beta_dot] = angularRates(Xdot, X);
        
        % Calculate errors in angle of attack and sideslip rates
        error_alpha_dot = abs((alpha_dot - alpha_dot_old)/alpha_dot_old);
        error_beta_dot = abs((beta_dot - beta_dot_old)/beta_dot_old);
        error = max([error_alpha_dot error_beta_dot]);
    
        % Store current angle of attack/sideslip rates for next iteration
        alpha_dot_old = alpha_dot;
        beta_dot_old = beta_dot;
    
        % Break with warning if iteration limit is reached
        if iterCount > iterLim
            warning('Reached iteration limit for alpha and beta rates!');
            break
        end
    
        % Incriment iteration counter
        iterCount = iterCount + 1;
    end  
end