% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460369684
% Main Script

%% File setup
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
[U_turn, ratio] = steadyTurnEstimate(Params, U_trimmed, V_trimmed);

% Print controls to screen
fprintf('------------------ Turn Control Estimate ------------------');
fprintf('\n');
fprintf('Thrust (percent): %s', num2str(U_turn(1)));
fprintf('\n');
fprintf('Elevator (rad): %s', num2str(U_turn(2)));
fprintf('\n');
fprintf('Aileron (rad): %s', num2str(U_turn(3)));
fprintf('\n');
fprintf('Rudder (rad): %s', num2str(U_turn(4)));
fprintf('\n');
fprintf('Ratio of Aileron to Rudder: %s', num2str(ratio));
fprintf('\n');
fprintf('-----------------------------------------------------------');
fprintf('\n');

%% Run simulation
% Create time vector
timeEnd = 110;
dt = 0.01;
time = 0:dt:timeEnd;

% Set initial state
X(:,1) = X_trimmed;
U(:,1) = U_trimmed;

Xdot_trimmed = getstaterates(Params, X_trimmed, U_trimmed);
disp(Xdot_trimmed)

beta = zeros(1,length(time));
alpha = zeros(1,length(time));
[~, alpha(1), beta(1)] = aeroangles(X(:,1));

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
        U_manoeurve = controls5(U_trimmed, time(i), U_filter, T_filter);
        
        % Determine new state
        [X_new] = rungeKutta4(Params,X(:,i-1),U_manoeurve,dt);
        
        % Save result
        X(:,i) = X_new;
        U(:,i) = U_manoeurve;
    end
    
    % Get sideslip and airspeed
    [V, alpha(i), beta(i)] = aeroangles(X(:,i));
    
%     % Analytical estimate of controls required for steady coordinated turn
%     U_turn(:,i) = steadyTurnEstimate(Params, U_manoeurve, V);
    
end

% % Plot results
% simulate(X)
testPlotControls5(X,U,time);

figure;
plot(time,rad2deg(beta));
grid on
xlabel('Time (s)');
ylabel('Sideslip Angle (deg)');
% figure;
% plot(time,rad2deg(alpha));
% grid on
% xlabel('Time (s)');
% ylabel('Angle of Attack (deg)');