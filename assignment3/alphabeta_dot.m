% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460398189
% AlphaBeta_dot
%
%
%
%
% Inputs: X_dot, X arrays (column vectors)
%
% Outputs:
% [alpha_dot, beta_dot]

function [alpha_dot, beta_dot] = alphabeta_dot(Xdot,X)

        % Calcualte alpha_dot 
        alpha_dot = (Xdot(3)*X(1) - X(3)*Xdot(1))/X(1);
        
        % Calculate beta_dot MUST CHANGE ------------
        beta_dot = (Xdot(3)*X(2) - X(2)*Xdot(2))/X(3);
end

