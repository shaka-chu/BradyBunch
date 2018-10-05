% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460369684
% Main Script

clear;
clc;
clf;
clf reset;
close all;

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

% Create time vector
timeEnd = 5;
dt = 0.01;
time = 0:dt:timeEnd;

% Set initial state
X(:,1) = X_trimmed;
U(:,1) = U_trimmed;

Xdot_trimmed = getstaterates(Params, X_trimmed, U_trimmed);
disp(Xdot_trimmed)

% % Loop through time vector
% for i = 2:length(time)
%     
%     % Run aircraft at trimmed settings for 1 second and then begin
%     % simulation
%     if time(i) <= 1
%         
%         % Determine new state
%         [X_new] = rungeKutta4(Params,X(:,i-1),U_trimmed,dt);
%         
%         % Save result
%         X(:,i) = X_new;
%         U(:,i) = U_trimmed;
%     
%     else
%         
%         % Determine control setting for manoeurve
%         U_manoeurve = controls(U_trimmed, time, time(i));
%         
%         % Determine new state
%         [X_new] = rungeKutta4(Params,X(:,i-1),U_manoeurve,dt);
%         
%         % Save result
%         X(:,i) = X_new;
%         U(:,i) = U_manoeurve;
%     end
%     
% end
% 
% % Plot results
% for j = 1:length(X(:,1))
%     figure(j)
%     plot(X(j,:),time);
% end