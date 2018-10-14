% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460369684
% Main Script

clf;
clf reset;
close all;
clear;
clc;

% Add every subfolder to path
folder = fileparts(which('Main.m')); 
addpath(genpath(folder));

% Run the control GUI
run Control_GUI_FULL
clc

% Plotting colors
red     = [0.8471 0.1176 0.1922];
blue    = [0.1059 0.3882 0.6157];
black   = [0 0 0];
green   = [0 0.5020 0];
cyan    = [0.0078 0.6627 0.8863];
yellow  = [0.9843 0.7608 0.0510];
gray    = [0.3490 0.3490 0.3490];
orange  = [1.0000 0.4980 0];
purple  = [0.5961 0.3059 0.6392];

% Plotting line width
lw = 1.2;

% Set plotting settings
set(groot,'defaultAxesColorOrder',[black;blue;red;green;yellow;cyan;...
    purple;orange]);
set(0,'defaultLineLineWidth',lw);

%% Simulation setup
% Choose CG positionn(1 or 2)
cgPos = 'CG1';

% Choose initial flight condition (3 or 4)
flightCond = 'COND180';

% Switch expression
situation = [cgPos, flightCond];

% Initialise aircraft parameters
[Nominal_params, Secondary_params] = initialisation;

% Load 
switch situation
    
    % CG 1, 100 kN 1000 ft
    case 'CG1COND100'
        
        % Load flight condition .mat file
        load ICs_PC9_nominalCG1_100Kn_1000ft
        
        % Obtain state and control vectors (after converting Euler angles
        % to quaternions!)
        X_initial = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
        U_initial = U0;
        
        % Set aircraft parameters
        Params = Nominal_params;
        
    % CG 1, 180 kN 1000 ft
    case 'CG1COND180'
        
        % Load flight condition .mat file
        load ICs_PC9_nominalCG1_180Kn_1000ft
        
        % Obtain state and control vectors (after converting Euler angles
        % to quaternions!)
        X_initial = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
        U_initial = U0;
        
        % Set aircraft parameters
        Params = Nominal_params;
    
    % CG 2, 100 kN 1000 ft
    case 'CG2COND100'
        
        % Load flight condition .mat file
        load ICs_PC9_CG2_100Kn_1000ft
        
        % Obtain state and control vectors (after converting Euler angles
        % to quaternions!)
        X_initial = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
        U_initial = U0;
        
        % Set aircraft parameters
        Params = Secondary_params;
        
    case 'CG2COND180'
        
        % Load flight condition .mat file
        load ICs_PC9_CG2_180Kn_1000ft
        
        % Obtain state and control vectors (after converting Euler angles
        % to quaternions!)
        X_initial = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
        U_initial = U0;
        
        % Set aircraft parameters
        Params = Secondary_params;
end

%% Check trim function
% Create test states
V_test1 = 120;
alt_test1 = convlength(5000, 'ft', 'm');
X_test1 = [V_test1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -alt_test1]';

V_test2 = 70;
alt_test2 = convlength(5000, 'ft', 'm');
X_test2 = [V_test2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -alt_test2]';

% Obtain test trimmed state to verify
[X_trimmedTest1, U_trimmedTest1] = trim(Params, X_test1);
[X_trimmedTest2, U_trimmedTest2] = trim(Params, X_test2);

% Verify the state rates are zero
Xdot_Test1 = getstaterates(Params, X_trimmedTest1, U_trimmedTest1);
Xdot_Test2 = getstaterates(Params, X_trimmedTest2, U_trimmedTest2);

% Display the test state rates
disp('==============================================================')
disp('State rates for test case 1')
disp('==============================================================')
disp(Xdot_Test1)
disp('==============================================================')
disp('State rates for test case 2')
disp('==============================================================')
disp(Xdot_Test2)
disp('==============================================================')

%% Trim aircraft
% Run trim function
[X_trimmed, U_trimmed] = trim(Params, X_initial);

%% Run simulation
% For the seven flight cases
timeEnd = [400 400 400 60 180 90];

for k = 1:6
    
    clear X U time U_filter T_filter
    clc
    close all
    
    % Create time vector
    dt = 0.01;
    time = 0:dt:timeEnd(k);

    % Set initial state
    X(:,1) = X_trimmed;
    U(:,1) = U_trimmed;
    
    % Load the flight controls .mat file
    disp('==============================================================')
    fprintf('Load ''control%d.mat'' to control GUI and into workspace', k)
    fprintf('\nthen press any key to continue\n')
    disp('==============================================================')
    pause
    clc

    % Loop through time vector
    for i = 2:length(time)

        % Run aircraft at trimmed settings for 1 second and then begin
        % simulation
        if time(i) <= 1

            % Determine new state
            [X_new] = rungeKutta4(Params, X(:,i-1), U_trimmed, dt);

            % Save result
            X(:,i) = X_new;
            U(:,i) = U_trimmed;
        else

            % Determine control setting for manoeurve
            U_manoeurve = controls(U_trimmed, time(i), ...
                U_filter, T_filter);

            % Determine new state
            [X_new] = rungeKutta4(Params, X(:,i-1), U_manoeurve, dt);

            % Save result
            X(:,i) = X_new;
            U(:,i) = U_manoeurve;
        end
    end

    % Plot results
    disp('==============================================================')
    fprintf('Manoeuvre %d\n', k)
    disp('==============================================================')
    fprintf('Press any key for the next manoeuvre\n')
    disp('==============================================================')
    
    if k > 3
        simulate(X)
    end
    plotData(X, U, time)
    pause
end