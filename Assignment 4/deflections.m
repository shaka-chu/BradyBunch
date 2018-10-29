% AERO3560 Assignmnet 4
% Author: 460306678


function [X_elevator,X_aileron, X_rudder] = deflections(X,time, A_lat, A_long, B_lat, B_long)

    % Loop through control deflection
    for i = 1:3

        % Call function
        [X_hist] = simulateResponse(X,time,i,A_lat, A_long, B_lat, B_long);

        % Save result
        if i == 1
            X_elevator = X_hist;
        elseif i == 2
            X_aileron = X_hist;
        elseif i == 3
            X_rudder = X_hist;
        end

    end
    
   
end