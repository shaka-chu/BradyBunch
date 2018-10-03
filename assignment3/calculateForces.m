function [BodyForces, gravForces, thrust] = calculateForces(Params, X, U)

    % Get aerodynamic angles
    [V, alpha, beta] = aeroangles(X);
    
    % Get flow properties
    [rho, Q]  = flowproperties(X, V);
    
    % Calculate gravity forces
    [Fgx, Fgy, Fgz] = gravity(Params, X);
    
    % Calculate wind forces
    [Cfa_z, Cfa_x, CL] = windforces(Params, alpha, X, U, V);
    
    % Calculate body forces
    alpha_dot = 0;      % TODO - calculate this
    beta_dot = 0;       % TODO - calculate this
    [F_body, M_body] = bodyforces(Params, X, U, Cfa_x, Cfa_z, CL, ...
                       Q, alpha, beta, alpha_dot, beta_dot,V);

    % Concatenate gravity forces into cell array
    gravForces = {Fgx, Fgy, Fgz};
    
    % Calculate thrust force
    thrust = propforce(Params, X, U, rho);

    % Save body forces to struct
    BodyForces.Force    = F_body;
    BodyForces.Moment   = M_body;
    
end