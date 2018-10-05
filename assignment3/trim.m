% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460373315
% Function Name: trim
%
%
% Function Description:
% Returns the forces acting on the aircraft in the body axes
%
% Inputs:
%   Params: Struct containing all characteristics of the aircraft
%
% Outputs:
%   trim_input : input control for trim [alpha; delta_t; delta_e]

function [X_trimmed, U_trimmed] = trim(Params, X0, U0)

    % Extract aircraft parameters
%     m           = Params.Inertial.m;
%     g           = Params.Inertial.g;
%     S           = Params.Geo.S;
%     CLa         = Params.Aero.CLa;
%     CLo         = Params.Aero.CLo;
    control_min = Params.ControlLimits.Lower;
    control_max = Params.ControlLimits.Upper;
    
    % Determine aircraft aerodynamic angles and airspeed
    [V, alpha] = aeroangles(X0);
    
%     % Determine the flow properties of the aircraft
%     [~, Q] = flowproperties(X0, V);
    
%     % Estimate the lift coefficient
%     CL = m*g/Q/S;
    
%     % Make initial estimates of the inputs
%     alpha0 = (CL - CLo)/CLa;
%     delta_t0 = 0.0001;
%     delta_e0 = 0.0001;
%     trim_input = [alpha0; delta_t0; delta_e0];
%     kPlus1 = trim_input;

    % Define indices in the state and state rate vector, to be
    % trimmed, u, w, and q. The rate of change of which should be zero
    % after trimming
    iTrim = [1 3 5];
    
    % Preallocate memory
    J = zeros(length(iTrim));
    
    % Define perturbation increment
    CHANGE = 1e-5;
    
    % Define the xbar vector, i.e. the values to be perturbed
    x_bar = [alpha; U0(1); U0(2)];
    
    % Initialise convergance boolean and tolerance
    converged = false;
    tol = 1e-9;
    
    % Numerical Newton-Ralphson method to solve for control inputs
    while ~converged        
          
        % Determine the state rate vector
        [Xdot] = getstaterates(Params, X0, U0);
        fx_bar = Xdot(iTrim);

        % Perturb the variables to get the Jacobian matrix
        for k = 1:length(x_bar)
            
            % Initialise the state and input vector to be trimmed
            X_new = X0;
            U_new = U0;
            
            % For the perturbation of alpha
            if k == 1
                
                % Perturbation of alpha, which affects u and w in the
                % state vector
                X_new(1) = V*cos(x_bar(1) + CHANGE);
                X_new(3) = V*sin(x_bar(1) + CHANGE);
               
            % For the perturbations of inputs
            else
                
                % Perturbation of the input vector, delta_t and
                % delta_e
                U_new(k-1) = U_new(k-1) + CHANGE;
            end
            
            % Determine the state rate vector for the perturbed state
            % and input vectors
            [Xdot_new] = getstaterates(Params, X_new, U_new);

            % Place in the first column of the Jacobian matrix
            J(:, k) = (Xdot_new(iTrim) - Xdot(iTrim))./CHANGE;
        end
        
        % Update the x_bar vector
        x_bar_new = x_bar - J\fx_bar;
        
        % Determine error
        error = abs((x_bar_new - x_bar)./x_bar);
        
        % Check if convergance condition is satisfied
        if max(error) < tol
            converged = true;
        end
        
        % Update the x_bar vector
        x_bar = x_bar_new;
        
        % Update the state and input vectors
        X0(1) = V*cos(x_bar(1));
        X0(3) = V*sin(x_bar(1));
        U0(1) = x_bar(2);
        U0(2) = x_bar(3);
        
        % Check if exceeding control limits
        if any(U0 > control_max) || any(U0 < control_min)
            
            % Return error for exceeding control limit
            error('Exceeding control limits')
        end
    end
    
    % Save the final state and input vectors as trimmed vectors
    X_trimmed = X0;
    U_trimmed = U0;
end