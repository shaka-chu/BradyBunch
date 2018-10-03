% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460373315
% Function Name: trim
%
%
% Function Description:
% Returns the forces acting on the aircraft in the body axes
%
% Inputs:
%
% Outputs:
%   trim_input : input control for trim [alpha; delta_t; delta_e]

function trim_input = trim(Params, V_trim, h, gamma, phi_0, theta_0, psi_0)

    % Extract aircraft parameters
    m   = Params.Inertial.m;
    g   = Params.Inertial.g;
    S   = Params.Geo.S;
    CLa = Params.Aero.CLa;
    CLo = Params.Aero.CLo;
    
    % Get the flow properties
    [~, Q] = flowproperties(h, V_trim);

    % Set tolorence of numerical convergance
    tol = 1e-9;
    
    % Estimate the lift coefficient
    CL = m*g/Q/S;
    
    % Make initial estimates of the inputs
    alpha0 = (CL - CLo)/CLa;
    delta_t0 = deg2rad(0.5);
    delta_e0 = 0;
    trim_input = [alpha0; delta_t0; delta_e0];
    iTrim = [1 3 5];
    J = zeros(length(iTrim));
    
    % Initialise the state vectors
    X = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -h]; 
    U = [delta_t0; delta_e0; 0; 0];
    
    % Initialise converged boolean
    notConverged = true;
    
    % Numerical Newton-Ralphson method to solve for control inputs
    while notConverged
        
        % Calculate the velocity components
        X(1) = V_trim*cos(trim_input(1));
        X(3) = V_trim*sin(trim_input(1));
        
        % Calculate quaternions from the initial Euler angles
        X(7:10) = euler2quat([phi_0; theta_0; psi_0]);
        
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
                Xk(1) = V_trim*cos(trim_input(1) + 0.001*trim_input(1));
                Xk(3) = V_trim*sin(trim_input(1) + 0.001*trim_input(1));
                
                % Determine the state rate vector
                [Xkdot] = getstaterates(Params, Xk, Uk);
                
                % Place in the first column of the Jacobian matrix
                J(:, k) = (Xkdot(iTrim) - Xdot(iTrim))./(0.001*alpha);
               
            % For the other perturbations
            else
                
                % Initilise state vector for peturbation
                Xk = X;
                Uk = U;
                
                % Change the inputs
                Uk(k-1) = U(k-1) + 0.001*U(k-1);
                
                % Determine the state rate vector
                [Xkdot] = getstaterates(Params, Xk, Uk);
                
                % Place in the first column of the Jacobian matrix
                J(:, k) = (Xkdot(iTrim) - Xdot(iTrim))./(0.001*U(k-1));
            end
        end
        
        % Evaluate the new inputs
        kPlus1 = trim_input - J\Xk_barDot;
        
        % Determine error
        error = (kPlus1 - trim_input)'*(kPlus1 - trim_input);
        
        % Check for convergance
        if error < tol
            notConverged = false;
        end
        
        % Update the input
        trim_input = kPlus1;
    end
end