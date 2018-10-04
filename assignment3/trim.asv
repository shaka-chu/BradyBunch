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

function trim_input = trim(Params, X, U0)

    % Extract aircraft parameters
    m           = Params.Inertial.m;
    g           = Params.Inertial.g;
    S           = Params.Geo.S;
    CLa         = Params.Aero.CLa;
    CLo         = Params.Aero.CLo;
    control_min = Params.ControlLimits.Lower;
    control_max = Params.ControlLimits.Upper;
    
    % Get the velocity of the aircraft
    [V_trim, alpha00] = aeroangles(X);
    
    % Get the flow properties
    [~, Q] = flowproperties(X, V_trim);
    
    % Set tolorence of numerical convergance
    tol = 1e-6;
    
    % Estimate the lift coefficient
    CL = m*g/Q/S;
    
    % Make initial estimates of the inputs
    alpha0 = (CL - CLo)/CLa;
    delta_t0 = 0.0001;
    delta_e0 = 0.0001;
    trim_input = [alpha0; delta_t0; delta_e0];
    kPlus1 = trim_input;
    iTrim = [1 3 5];
    J = zeros(length(iTrim));
    trim_input = [alpha00; U0(1); U0(2)];
    
    CHANGE = 1e-5;
    
    
    % Initialise the state vectors
    U = [delta_t0; delta_e0; 0; 0];
    U = [U0(1); U0(2); 0; 0];
    
    % Initialise converged boolean
    notConverged = true;
    
    % Numerical Newton-Ralphson method to solve for control inputs
    while notConverged
        
        % Normalise the quaternions
        
          
        % Determine the state rate vector
        [Xdot] = getstaterates(Params, X, U);
        Xk_barDot = Xdot(iTrim);

        % Perturb the variables to get the Jacobian matrix
        for k = 1:length(trim_input)
            
            % For the perturbation of alpha
            if k == 1
                
                % Initilise state vector for peturbation
                Xk = X;
                Uk = U;
                
                % Change the u and w
                Xk(1) = V_trim*cos(trim_input(1) + CHANGE);
                Xk(3) = V_trim*sin(trim_input(1) + CHANGE);
                
                % Determine the state rate vector
                [Xkdot] = getstaterates(Params, Xk, Uk);
                
                % Place in the first column of the Jacobian matrix
                J(:, k) = (Xkdot(iTrim) - Xdot(iTrim))./CHANGE;
               
            % For the other perturbations
            else
                
                % Initilise state vector for peturbation
                Xk = X;
                Uk = U;
                
                % Change the inputs
                Uk(k-1) = U(k-1) + CHANGE;
                
                % Determine the state rate vector
                [Xkdot] = getstaterates(Params, Xk, Uk);
                
                % Place in the first column of the Jacobian matrix
                J(:, k) = (Xkdot(iTrim) - Xdot(iTrim))./CHANGE;
            end
        end
        
        % Evaluate the new inputs
        kPlus1 = trim_input - J\Xk_barDot;
        
        % Determine error
        error = (kPlus1 - trim_input)'*(kPlus1 - trim_input);
        error = sum(abs(kPlus1 - trim_input));
        
        % Check for convergance
        if error < tol
            notConverged = false;
        end
        
        % Update the input
        trim_input = kPlus1;
    end
end