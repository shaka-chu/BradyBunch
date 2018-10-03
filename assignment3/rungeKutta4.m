function [X] = rungeKutta4(Params,X0,U0,dt)
    
    X_dot_1 = staterates(X_0, U_0, Params)
    
    An = X_dot_1*dt/2;
    
    X_2 = X_0 + An
    
    
    
    X_dot_2 = staterates(X_2, U_0, Params,)

     Ft, F_body, M_body, Fgx, Fgy, Fgz

    % Step 1
    An = Xdot*dt;
    
    
    
    

end