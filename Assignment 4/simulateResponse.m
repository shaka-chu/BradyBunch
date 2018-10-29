% AERO3560 Assignment 4 Part B
% Author: 460306678

function [X] = simulateResponse(X0,time,control, A_lat, A_long, B_lat, B_long)
    
    % Initialise control
    if control == 1
        Udeflect = [0.5; deg2rad(5); 0 ; 0];
    elseif control == 2
        Udeflect = [0.5; 0; deg2rad(5); 0];
    elseif control == 3
        Udeflect = [0.5; 0 ; 0 ; deg2rad(5)];
    end
    
    % Initialise
    X(:,1) = X0;
    
    % Determine time-step
    dt = time(2) - time(1);
    
    % Loop through time vectorzx
    for i = 2:length(time)

        % Determine control settings
        if time(i) >= 1 && time(i) < 1 + 0.5
            U = Udeflect;
        else
            U = [0.5; 0 ; 0 ; 0];
        end

        % Break state into lateral and longitudinal components
        X_long = X(1:5,i-1);
        X_lat = X(6:10,i-1);

        % Break control into lateral and longitudinal components
        U_lat = U(3:4);
        U_long = U(1:2);

        % Find lateral rates of change
        Lat_dot = A_lat*X_lat + B_lat*U_lat;

        % Find longitudinal rates of change
        Long_dot = A_long*X_long + B_long*U_long;

        % Apply lateral Euler integration
        X_lat = X_lat + Lat_dot*dt;

        % Apply longitudinal Euler integrationgit
        X_long = X_long + Long_dot*dt;

        % Store full state
        X(:,i) = [X_long;X_lat];

    end

end