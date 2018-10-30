% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678, 460369684, 460373315, 460369189
% Function Name: plotData
%
% Function Description:
%   Takes the state and control vector after the simulation and plots
%   results against time
%
% Inputs:
%   varagin: Variable length input
%
% Outputs: none
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

function plotData(varargin)

    % Check the variable length input
    if length(varargin) == 5
        
        % Extract the inputs
        X = varargin{1};
        U = varargin{2};
        time = varargin{3};
        toSave = varargin{4};
        controlSuffix = varargin{5};
        plotU = true;
        
        % Get save directory
        if toSave
            currentFolder = pwd;
            saveLoc = uigetdir(currentFolder, 'Select save folder');
        end        
    elseif length(varargin) == 2
        
        % Extract the inputs
        X = varargin{1};
        time = varargin{2};
        toSave = false;
        plotU = false;
    else
        
        % Extract the inputs
        X = varargin{1};
        U = varargin{2};
        time = varargin{3};
        toSave = false;
        plotU = true;
    end
    
    % Create velocity figure
    fig1 = figure;
    x0=100;
    y0=100;
    width=550;
    height=500;
    set(gcf,'units','points','position',[x0,y0,width,height])
    subplot(4,1,1)
    plot(time, sqrt(X(1,:).^2 + X(2,:).^2 + X(3,:).^2), 'LineWidth',2)
    title('V')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    grid on
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
    subplot(4,1,2)
    plot(time, X(1,:), 'LineWidth',2)
    title('u')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    grid on
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
    subplot(4,1,3)
    plot(time, X(2,:), 'LineWidth',2)
    title('v')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    grid on
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
    subplot(4,1,4)
    plot(time, X(3,:), 'LineWidth',2)
    title('w')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    grid on
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
    % Save figure into selected folder
    if toSave
        print(fig1, [saveLoc filesep 'velocityFigure_' ...
            controlSuffix], '-depsc')
    end
    
    % Create body rates figure
    fig2 = figure;
    subplot(3,1,1)
    plot(time, rad2deg(X(4,:)), 'LineWidth',2)
    title('p')
    xlabel('Time (s)')
    ylabel('Rate (\circ/s)')
    grid on
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
    subplot(3,1,2)
    plot(time, rad2deg(X(5,:)), 'LineWidth',2)
    title('q')
    xlabel('Time (s)')
    ylabel('Rate (\circ/s)')
    grid on
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
    subplot(3,1,3)
    plot(time, rad2deg(X(6,:)), 'LineWidth',2)
    title('r')
    xlabel('Time (s)')
    ylabel('Rate (\circ/s)')
    grid on
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
    % Save figure into selected folder
    if toSave
        print(fig2, [saveLoc filesep 'bodyRatesFigure_' ...
            controlSuffix], '-depsc')
    end
    
    % Create attitude figure
    euler = rad2deg(quat2euler(X(7:10,:)));
    fig3 = figure;
    plot(time, euler(1,:), time, euler(2,:), time, euler(3,:), 'LineWidth',2)
    xlabel('Time (s)')
    ylabel('Attitude (\circ)')
    grid on
    h = legend('Bank angle, $\theta$','Pitch angle, $\phi$','Yaw angle, $\psi$');
    set(h,'Interpreter','latex');
    set(h,'location','best');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
    % Save figure into selected folder
    if toSave
        print(fig3, [saveLoc filesep 'attitudeFigure_' ...
            controlSuffix], '-depsc')
    end
    
    % Create position figure
    fig4 = figure;
    if plotU
        
        plot(time, X(11,:), time, X(12,:), time, -X(13,:), 'LineWidth',2)
        xlabel('Time (s)')
        ylabel('Position (m)')
        grid on
        h = legend('X-position', 'Y-position','Height');
        set(h,'Interpreter','latex');
        set(h,'location','best');
        set(gcf, 'Color', [1 1 1]);
        set(gca, 'Color', [1 1 1]);
    else
        plot(time, -X(13,:), 'LineWidth',2)
        xlabel('Time (s)')
        ylabel('Height (m)')
        grid on
        set(gcf, 'Color', [1 1 1]);
        set(gca, 'Color', [1 1 1]);
    end
    
    % Save figure into selected folder
    if toSave
        print(fig4, [saveLoc filesep 'positionFigure_' ...
            controlSuffix], '-depsc')
    end
    
    % Create control figure
    if plotU
        fig5 = figure;
        subplot(2,2,1)
        plot(time, U(1,:), 'LineWidth',2)
        title('Throttle')
        xlabel('Time (s)')
        ylabel('Throttle Fraction')
        grid on
        set(gcf, 'Color', [1 1 1]);
        set(gca, 'Color', [1 1 1]);

        subplot(2,2,2)
        plot(time, rad2deg(U(2,:)), 'LineWidth',2)
        title('Elevator')
        xlabel('Time (s)')
        ylabel('Deflection (\circ)')
        grid on
        set(gcf, 'Color', [1 1 1]);
        set(gca, 'Color', [1 1 1]);

        subplot(2,2,3)
        plot(time, rad2deg(U(3,:)), 'LineWidth',2)
        title('Ailerons')
        xlabel('Time (s)')
        ylabel('Deflection (\circ)')
        grid on
        set(gcf, 'Color', [1 1 1]);
        set(gca, 'Color', [1 1 1]);

        subplot(2,2,4)
        plot(time, rad2deg(U(4,:)), 'LineWidth',2)
        title('Rudder')
        xlabel('Time (s)')
        ylabel('Deflection (\circ)')
        grid on
        set(gcf, 'Color', [1 1 1]);
        set(gca, 'Color', [1 1 1]);

        % Save figure into selected folder
        if toSave
            print(fig5, [saveLoc filesep 'controlFigure_' ...
                controlSuffix], '-depsc')
        end
    end
end