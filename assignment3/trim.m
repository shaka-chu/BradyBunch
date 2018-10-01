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

function trim(Params, V_trim, h, gamma, phi_0, theta_0, psi_0)

    % Extract aircraft parameters

    % Set tolorence of numerical convergance
    tol = 1e-9;
    
    % Make initial estimates of the inputs
    alpha0 = deg2rad(2);
    delta_t0 = deg2rad(0.5);
    delta_e0 = 0;
    
    % Initialise the state vectors
    X = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; h]; 
    U = [delta_t0; delta_e0; 0; 0];
    
    % Initialise converged boolean
    notConverged = true;
    
    % Numerical Newton-Ralphson method to solve for control inputs
    while notConverged
        
        % Calculate the velocity components
        X(1) = V_trim*cos(alpha);
        X(3) = V_trim*sin(alpha);
        
        % Calculate quaternions from the initial Euler angles
        X(7:10) = euler2quat([phi_0; theta_0; psi_0]);
        
        % Determine the state rate vector
        % Determine aero angles from aeroangles.m function
        [V, alpha, beta] = aeroangles(X(1), X(2), X(3));
        
        % Determine flow properties from flowproperties.m function
        [rho, Q] = flowproperties(V, h);
        
        % Determine gravitational forces in the body fram from the
        % gravity.m function
        [Fgx, Fgy, Fgz] = gravity(Params, X(7), X(8), X(9), X(10));
        
        % Determine wind forces
        [Cfa_z, Cfa_x] = windforces(Params, alpha, X(5), U(2));
        
        % Determine propulsive forces
        [thrust] = propforce(Params, rho, X(1), U(1));
        
        % Determine body forces
        [F_body, M_body] = bodyforces(Params, Cfa_x, Cfa_z, CL, ...
            Q, alpha, beta, alpha_dot, beta_dot, delta_a, delta_e, ...
            delta_r, p_hat, q_hat, r_hat, phi);
    end
end