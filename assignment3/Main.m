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

% Initialise aircraft parameters
[Nominal_params, Secondary_params] = initialisation;

% Select CG position
Params = Nominal_params;

% Set initial heading
phi_0 = 0;
psi_0 = 0;
theta_0 = 0;
euler = [phi_0; theta_0; psi_0];
quaternion_0 = euler2quat(euler);

% Set flight conditions
V = 120;
h = convlength(5000, 'ft','m');

% Create initial state
X0 = [V; 0; 0; 0 ; 0; 0; quaternion_0; 0; 0; -h];

% Trim aircraft
[X_trimmed, U_trimmed] = trim(Params, X0);

%%
% Create time vector

timeEnd = 30;

dt = 0.01;
time = 0:dt:timeEnd;

% Set initial state
X(:,1) = X_trimmed;
U(:,1) = U_trimmed;

Xdot_trimmed = getstaterates(Params, X_trimmed, U_trimmed);
disp(Xdot_trimmed)

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
        U_manoeurve = controls4(Params, X(:,i-1), U_trimmed, time(i), U_linear, T_linear);
        
        % Determine new state
        [X_new] = rungeKutta4(Params,X(:,i-1),U_manoeurve,dt);
        
        % Save result
        X(:,i) = X_new;
        U(:,i) = U_manoeurve;
    end
    
end

% % Plot results
% simulate(X)
plotData(X,U,time)


