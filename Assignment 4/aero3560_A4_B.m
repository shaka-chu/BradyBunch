% Clear command window and reset figure properties
clear;
clc;
clf;
clf reset;
close all;

% Plotting settings
plotSettings

% Flight condition

% airspeed    = 'cruise';
airspeed    = 'approach';
% cgPos       = '2';
cgPos       = '1';


flightCond  = [airspeed cgPos];

% Obtain longitudinal-directional state space model and aircraft properties
[Alon, Blon, Params] = longitudinalStateSpace(flightCond) ;

% Flight speed
V = convvel(90,'kts','m/s');

% Density at flight speed
h               = convlength(500,'ft','m');
[~, ~, ~, rho]  = atmosisa(h);

% Pitch angle at steady level flight (radians)
theta = 0;

% Obtain lateral-directional state space model
[Alat, Blat] = lateralStateSpace(Params, V, theta, h);

% Do you want to print the eigen analysis to the command window?
printAnalysis = true;

% Do you want to plot the eigenvectors?
plsPlotEigenVecs = false;

% Do you want to plot root locus?
plsPlotRootLocus = false;

% Eigen analysis
EigAnalysis = eigenAnalysis(Alon, Alat, printAnalysis, ...
    plsPlotEigenVecs, plsPlotRootLocus);

% Create time vector for simulation
t_end = 1000;
dt = 0.01;
time = 0:dt:t_end;

% Create state vector X = [u w q theta z v p r phi psi]'
X = [0 0 0 0 0 0 0 0 0 0]';

% Calculate time histories
[X_elevator,X_aileron, X_rudder] = deflections(X, time, Alat, Alon , ...
    Blat, Blon);

% Do you want to plot the results?
plotResults = false;

% Results plotting (timeseries)
plotTimeSeries(V, X_elevator, time, 300, plotResults);
% plotTimeSeries(V, X_aileron, time, 150, plotResults);
% plotTimeSeries(V, X_rudder, time, 150, plotResults);

% Handling qualities
handlingQualities(Params, V, h, EigAnalysis)