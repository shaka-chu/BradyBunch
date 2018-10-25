% Clear command window and reset figure properties
clear;
clc;
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

%% Setup
% Call validation case function
validationCase;

% Lift curve slope (rad^-1)
A0.naca0012     = 1.4/(deg2rad(13.189));    % NACA0012
A0.naca65415    = 1.6/(deg2rad(15));        % NACA 65-415

% Zero lift AoA (rad)
Alpha0.naca0012     = 0;                    % NACA0012
Alpha0.naca65415    = -deg2rad(2);          % NACA 65-415

% Wing, tail and fuselage geometry
[WingProps, TailProps, FuseProps] = aircraftProps;

% Load experimental data files
[Model, AoA, U] = loadExperiment;

% Aircraft angles of attack (rad)
alpha = AoA.Radians;

% Number of control points
nPts = 1000;

% Generate vector of odd numbers (symmetrical wing/tailplane)
n = 2*(1:nPts) - 1;

%% Question 3
% Call function for parts 1, 2 and 3 of Q3
parts123(n, nPts, U, AoA, Alpha0, A0, WingProps, TailProps, Model, ...
    FuseProps)

% Call function for climb
climbCondition(n, nPts, U, Alpha0, A0, WingProps, TailProps, Model)

% Call function for banked turn
bankCondition(nPts, U, Alpha0, A0, WingProps, TailProps, Model)