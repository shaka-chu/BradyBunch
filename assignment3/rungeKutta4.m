function [X_new] = rungeKutta4(Params,X0,U,dt)
    
    % State rate at start of step
    X_dot_1 = staterates(Params, X0, U);
    
    % Incriment 
    An = X_dot_1*dt;
    
    % First state at middle of step
    X_2 = X_0 + An/2;
    
    % First state rate at middle of step
    X_dot_2 = staterates(Params, X_2, U);
    
    % Improved incriment 
    Bn = X_dot_2*dt;
    
    % Second state at middle of step
    X_3 = X_0 + Bn/2;
    
    % Second state rate at middle of step
    X_dot_3 = staterates(Params, X_3, U);
    
    % Improved incriment
    Cn = X_dot_3*dt;
    
    % State at end of step
    X_4 = X_0 + Cn;
    
    % State rate at end of step
    X_dot_4 = staterates(Params, X_4, U);
    
    % Improved incriment
    Dn = X_dot_4*dt;
    
    % Evaluate new state
    X_new = X_0 + (1/6)*(An + 2*Bn + 2*Cn + Dn);
end