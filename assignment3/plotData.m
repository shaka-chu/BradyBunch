% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: plotData
%
% Function Description:
%   Takes the state and control vector after the simulation and plots
%   results against time
%
% Inputs:
%   X:      Vector containing the aircraft state. The order is:
%               - u   = X(1)    (m/s)
%               - v   = X(2)    (m/s)
%               - w   = X(3)    (m/s)
%               - p   = X(4)    (rad/s)
%               - q   = X(5)    (rad/s)
%               - r   = X(6)    (rad/s)
%               - q0  = X(7)    -
%               - q1  = X(8)    -
%               - q2  = X(9)    -
%               - q3  = X(10)   -
%               - x   = X(11)   (m)
%               - y   = X(12)   (m)
%               - z   = X(13)   (m)
%   U:          Vector containing all aircraft control settings. The order is:
%                   - delta_t = U(1)    -
%                   - delta_e = U(2)    (rad)
%                   - delta_a = U(3)    (rad)
%                   - delta_r = U(4)    (rad)
%   Time:       Time simulation vector 
%
% Outputs:
%   None
%
% Other m-files required:
%   None
%
% Subfunctions:
%   None
%
% MAT-files required: none
%
% TODO: 
%   None

function plotData(X,U,time)

    % Create velocity figure
    figure(1)
    subplot(2,2,1)
    plot(time, X(1,:), 'LineWidth',2)
    title('u')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    grid on
    
    subplot(2,2,2)
    plot(time, X(2,:), 'LineWidth',2)
    title('v')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    grid on
    
    subplot(2,2,3)
    plot(time, X(3,:), 'LineWidth',2)
    title('w')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    grid on
    
    subplot(2,2,4)
    plot(time, sqrt(X(1,:).^2 + X(2,:).^2 + X(3,:).^2), 'LineWidth',2)
    title('V')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    grid on
    
    % Create body rates figure
    figure(2)
    subplot(3,1,1)
    plot(time, rad2deg(X(4,:)), 'LineWidth',2)
    title('p')
    xlabel('Time (s)')
    ylabel('Rate (\circ/s)')
    grid on
    
    subplot(3,1,2)
    plot(time, rad2deg(X(5,:)), 'LineWidth',2)
    title('q')
    xlabel('Time (s)')
    ylabel('Rate (\circ/s)')
    grid on
    
    subplot(3,1,3)
    plot(time, rad2deg(X(6,:)), 'LineWidth',2)
    title('r')
    xlabel('Time (s)')
    ylabel('Rate (\circ/s)')
    grid on
    
    % Create attitude figure
    euler = rad2deg(quat2euler(X(7:10,:)));
    figure(3)
    plot(time, euler(1,:), time, euler(2,:), time, euler(3,:), 'LineWidth',2)
    xlabel('Time (s)')
    ylabel('Attitude (\circ)')
    grid on
    h = legend('Bank angle, $\theta$','Pitch angle, $\phi$','Yaw angle, $\psi$');
    set(h,'Interpreter','latex');
    set(h,'location','best');
    
    % Create position figure
    figure(4)
    plot(time, X(11,:), time, X(12,:), time, -X(13,:), 'LineWidth',2)
    xlabel('Time (s)')
    ylabel('Position (m)')
    grid on
    h = legend('X-position', 'Y-position','Height');
    set(h,'Interpreter','latex');
    set(h,'location','best');
    
    % Create control figure
    figure(5)
    subplot(2,2,1)
    plot(time, U(1,:), 'LineWidth',2)
    title('Throttle')
    xlabel('Time (s)')
    ylabel('Throttle Fraction')
    grid on
    
    subplot(2,2,2)
    plot(time, rad2deg(U(2,:)), 'LineWidth',2)
    title('Elevator')
    xlabel('Time (s)')
    ylabel('Deflection (\circ)')
    grid on
    
    subplot(2,2,3)
    plot(time, rad2deg(U(3,:)), 'LineWidth',2)
    title('Ailerons')
    xlabel('Time (s)')
    ylabel('Deflection (\circ)')
    grid on
    
    subplot(2,2,4)
    plot(time, rad2deg(U(4,:)), 'LineWidth',2)
    title('Rudder')
    xlabel('Time (s)')
    ylabel('Deflection (\circ)')
    grid on

end