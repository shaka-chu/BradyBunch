% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460398189
% Alphabeta_dot
%
%
%
%
% Inputs: X_dot, X arrays (column vectors)
%
% Outputs:
% alpha_dot : Single value for alpha_dot using trhe following 
% alphadot = (wdot*u-w*udot)/(u^2)
% 
% [udot;vdot;wdot;pdot;qdot;rdot;q0dot;q1dot;q2dot;q3dot;position(1);position(2);position(3)]

function [alpha_dot] = alphabeta_dot(Xdot,X)

        % Calcualte alpha_dot 
        alpha_dot = (Xdot(3)*X(3) - X(1)*Xdot(1))/X(3);

end

