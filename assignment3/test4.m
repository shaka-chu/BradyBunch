% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460369684
% Main Script

clf;
clf reset;
close all;

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

% Choose initial flight condition (100kts or 180kts)
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

%% Trim aircraft
% Run trim function
[X_trimmed, U_trimmed] = trim(Params, X_initial);

%% Estimate control inputs required for steady turn at the trim condition
% Obtain trim airspeed
[V_trimmed, ~, ~] = aeroangles(X_trimmed);

% Call estimation function
U_turn = steadyTurnEstimate(Params, U_trimmed, V_trimmed);

% Print controls to screen
fpr
fprintf('Thrust: %s', num2str(U_turn(1)));

%% Run simulation
% Create time vector
timeEnd = 60;
dt = 0.01;
time = 0:dt:timeEnd;

% Set initial state
X(:,1) = X_trimmed;
U(:,1) = U_trimmed;

Xdot_trimmed = getstaterates(Params, X_trimmed, U_trimmed);
disp(Xdot_trimmed)

beta = zeros(1,length(time));
<<<<<<< HEAD:assignment3/TestControls5.asv
alpha = zeros(1,length(time));
[~, alpha(1), beta(1)] = aeroangles(X(:,1));
=======
[~, ~, beta(1)] = aeroangles(X(:,1));
>>>>>>> 83c9924317cd8249aafd4bd6cf3c3603ea4be4cf:assignment3/test4.m

% Loop through time vector
for i = 2:length(time)
    
    % Run aircraft at trimmed settings for 1 second and then begin
    % simulation
    if time(i) <= 1
        
        % Determine new state
        [X_new] = rungeKutta4(Params,X(:,i-1),U_trimmed,dt);
        
        % Save result
        X(:,i) = X_new;
        U(:,i) = U_trimmed;
    
    else
        
        % Determine control setting for manoeurve

        U_manoeurve = controls4(Params, X(:,i-1), U_trimmed, time(i), U_filter, T_linear);

        
        % Determine new state
        [X_new] = rungeKutta4(Params,X(:,i-1),U_manoeurve,dt);
        
        % Save result
        X(:,i) = X_new;
        U(:,i) = U_manoeurve;
        
        [~,alpha(i),beta(i)] = aeroangles(X(:,i));
    end
    
<<<<<<< HEAD:assignment3/TestControls5.asv
    % Get sideslip and airspeed
    [V, alpha(i), beta(i)] = aeroangles(X(:,i));
    
%     % Analytical estimate of controls required for steady coordinated turn
%     U_turn(:,i) = steadyTurnEstimate(Params, U_manoeurve, V);
    
=======
>>>>>>> 83c9924317cd8249aafd4bd6cf3c3603ea4be4cf:assignment3/test4.m
end

% % Plot results
simulate(X)

plotData(X,U,time)


figure
plot(time, rad2deg(alpha));
grid on
<<<<<<< HEAD:assignment3/TestControls5.asv
xlabel('Time (s)');
ylabel('Sideslip Angle (deg)');
% figure;
% plot(time,rad2deg(alpha));
% grid on
% xlabel('Time (s)');
% ylabel('Angle of Attack (deg)');
=======
xlabel('Time (s)')
ylabel('AOA (deg')

manoeurve4(X,time)

>>>>>>> 83c9924317cd8249aafd4bd6cf3c3603ea4be4cf:assignment3/test4.m
