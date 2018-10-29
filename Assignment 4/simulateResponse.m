% AERO3560 Assignment 4 Part B
% Author: 460306678

% Create time vector
t_end = 10;
dt = 0.01;
time = 0:dt:t_end;

function [X_hist] = simulateResponse(X,time,control)

    % Initialise control
    if control == 1
        Udeflect = [1; deg2rad(5); 0 ; 0];
    elseif control == 2
        Udeflect = [1; 0; deg2rad(5); 0];
    elseif control == 3
        Udeflect = [1; 0 ; 0 ; deg2rad(5)];
    end
    m = length(Udeflect);

    % Initialise vectors
    n = length(X);
    T = zeros(1,length(time));
    X = zeros(n,length(time));
    U = zeros(m,length(time));
    
    % Break state into lateral and longitudinal components
    X_lat = X(1:5,i-1);
    X_long = X(1:6,i-1);
    
    % Break control into lateral and longitudinal components
    U_lat = U(3:4,i-1);
    U_long = U(1:2,i-1);
    
    % Find lateral rates of change
    Lat_dot = A_lat*X_lat + B_lat*U_lat;
    
    % Find longitudinal rates of change
    Long_dot = A_long*X_long + B_long*U_long;
    
    % Apply lateral Euler integration
    X_lat = X_lat + Lat_dot*dt;
    
    % Apply longitudinal Euler integration
    X_long = X_long + Long_dot*dt;
    
    % Store full state
    X(:,i) = [X_lat; X_long];
    
    % Determine time-step
    dt = time(2) - time(1);
    
    % Loop through time vectorzx
    for i = 2:length(time)

        % Determine control settings
        if time(i) => 1 && time(i) < 1 + 0.5
            U = Udeflect;
        else
            U = [1; 0 ; 0 ; 0];
        end

        % Break state into lateral and longitudinal components
        X_lat = X(1:5,i-1);
        X_long = X(1:6,i-1);

        % Break control into lateral and longitudinal components
        U_lat = U(3:4,i-1);
        U_long = U(1:2,i-1);

        % Find lateral rates of change
        Lat_dot = A_lat*X_lat + B_lat*U_lat;

        % Find longitudinal rates of change
        Long_dot = A_long*X_long + B_long*U_long;

        % Apply lateral Euler integration
        X_lat = X_lat + Lat_dot*dt;

        % Apply longitudinal Euler integration
        X_long = X_long + Long_dot*dt;

        % Store full state
        X_hist(:,i) = [X_lat;X_long];

    end

end