% Clear command window and reset figure properties
clear;
clc;
clf;
clf reset;
close all;

% Plotting settings
plotSettings

% Flight condition
airspeed    = 'cruise';
%airspeed    = 'approach';
%cgPos       = '2';
cgPos       = '1';
flightCond  = [airspeed cgPos];

% Obtain longitudinal-directional state space model and aircraft properties
[Alon, Blon, Params] = longitudinalStateSpace(flightCond) ;

% Flight speed
V = convvel(220,'kts','m/s');

% Density at flight speed
h               = convlength(500,'ft','m');
[~, ~, ~, rho]  = atmosisa(h);

% Pitch angle at steady level flight (radians)
theta = 0;

% Obtain lateral-directional state space model
[Alat, Blat] = lateralStateSpace(Params, V, theta, h);

% Do you want to print the eigen analysis to the command window?
printAnalysis = true;

% Eigen analysis
EigAnalysis = eigenAnalysis(Alon, Alat, printAnalysis);

% Create time vector for simulation
t_end = 300;
dt = 0.01;
time = 0:dt:t_end;

% Create state vector X = [u w q theta z v p r phi psi]'
X = [0 0 0 0 0 0 0 0 0 0]';

% Calculate time histories
[X_elevator,X_aileron, X_rudder] = deflections(X,time, Alat,Alon,Blat, Blon);

% Do you want to plot the results?
plotResults = true;

% Results plotting (timeseries)
plotTimeSeries(X_elevator, X_aileron, X_rudder, time, plotResults);
