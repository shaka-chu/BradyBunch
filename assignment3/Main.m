% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460369684
% Main Script

% Initialise aircraft parameters
[Nominal_params, Secondary_params] = initialisation;

% Set initial heading
phi_0 = 0;
psi_0 = 0;
theta_0 = 0;
gamma = 0;

% Set trim conditions
V = 120;
h = convlength(5000, 'ft','m');

% Trim aircraft
trim_input = trim(Nominal_params, V, h, gamma, phi_0, theta_0, psi_0);