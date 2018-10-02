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
%   F_body : matrix of the combined body forces [F_bx; F_by; F_bz] 
%   M_body : matrix of the combined moments [M_bL; M_bM; M_bN]

function trim_input = trim(Params, V_trim, h, gamma, phi_0, theta_0, psi_0)

    % Extract aircraft parameters

    % Set tolorence of numerical convergance
    tol = 1e-9;
    
    % Make initial estimates of the inputs
    alpha0 = deg2rad(2);
    delta_t0 = deg2rad(0.5);
    delta_e0 = 0;
    trim_input = [alpha0 delta_t0 delta_e0];
    iTrim = [1 14 15];
    
    % Initialise the state vectors
    X = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; h]; 
    U = [delta_t0; delta_e0; 0; 0];
    X_aug = [X; U];
    
    % Initialise converged boolean
    notConverged = true;
    
    % Numerical Newton-Ralphson method to solve for control inputs
    while notConverged
        
        % Calculate the velocity components
        X(1) = V_trim*cos(alpha);
        X(3) = V_trim*sin(alpha);
        
        % Calculate quaternions from the initial Euler angles
        X(7:10) = euler2quat([phi_0; theta_0; psi_0]);
        
        % Determine aero angles from aeroangles.m function
        [V, alpha, beta] = aeroangles(X(1), X(2), X(3));
        
        % Determine flow properties from flowproperties.m function
        [rho, Q] = flowproperties(V, h);
        
        % Determine gravitational forces in the body fram from the
        % gravity.m function
        [Fgx, Fgy, Fgz] = gravity(Params, X(7), X(8), X(9), X(10));
        
        % Determine wind forces
        [Cfa_z, Cfa_x, CL] = windforces(Params, alpha, X(5), U(2));
        
        % Determine propulsive forces
        [thrust] = propforce(Params, rho, X(1), U(1));
        
        % Determine body forces
        [F_body, M_body] = bodyforces(Params, Cfa_x, Cfa_z, CL, ...
            Q, alpha, beta, 0, 0, U(1), U(2), U(4), ...
            X(4), X(5), X(6), phi_0);
        
        % Determine the state rate vector
        [Xdot] = staterates(X, Params, Ft, F_body, M_body, ...
            Fgx, Fgy, Fgz);

        % Perturb the variables to get the Jacobian matrix
        for k = 1:length(trim_input)
            
            % For the perturbation of alpha
            if k == 1
                
                % Change the u and w
                X(iTrim(k))
            end
        end
        
        
        % Perturb the state vector
        X_kPlus1 = X + Xdot;
        
    end
end