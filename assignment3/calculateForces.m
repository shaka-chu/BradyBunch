function [BodyForces, gravForces] = calculateForces(Params, X, U)

    % Get aerodynamic angles
    [V, alpha, beta] = aeroangles(x);
    
    % Get flow properties
    [~, Q]  = flowproperties(X, V);
    
    % Calculate gravity forces
    [Fgx, Fgy, Fgz] = gravity(Params, X);
    
    % Calculate wind forces
    [Cfa_z, Cfa_x, CL] = windforces(Params, alpha, X, U, V);
    
    % Calculate body forces
    [F_body, M_body] = bodyforces(Params, X, U, Cfa_x, Cfa_z, CL, ...
                       Q, alpha, beta, alpha_dot, beta_dot);

    % Concatenate gravity forces into cell array
    gravForces = {Fgx, Fgy, Fgz};

    % Save body forces to struct
    BodyForces.Force    = F_body;
    BodyForces.Moment   = M_body;
end