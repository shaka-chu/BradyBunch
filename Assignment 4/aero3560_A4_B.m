% Clear command window and reset figure properties
clear;
clc;
clf;
clf reset;
close all;

% Flight condition
airspeed    = 'cruise';
% airspeed    = 'approach';
cgPos       = '2';
% cgPos       = '1';
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
[X_elevator,X_aileron, X_rudder] = deflections(X,time, Alat,Alon ,Blat, Blon);


X_elevatorPlot = [X_elevator(1, :); X_elevator(6, :); X_elevator(2, :); ...
    X_elevator(7, :); X_elevator(3, :); X_elevator(8, :); ....
    euler2quat([X_elevator(9, :); X_elevator(4, :); X_elevator(10, :)]); ...
    zeros(size(X_elevator(1, :))); zeros(size(X_elevator(1, :))); ...
    X_elevator(5, :)];
X_aileronPlot = [X_aileron(1, :); X_aileron(6, :); X_aileron(2, :); ...
    X_aileron(7, :); X_aileron(3, :); X_aileron(8, :); ....
    euler2quat([X_aileron(9, :); X_aileron(4, :); X_aileron(10, :)]); ...
    zeros(size(X_aileron(1, :))); zeros(size(X_aileron(1, :))); ...
    X_aileron(5, :)];
X_rudderPlot = [X_rudder(1, :); X_rudder(6, :); X_rudder(2, :); ...
    X_rudder(7, :); X_rudder(3, :); X_rudder(8, :); ....
    euler2quat([X_rudder(9, :); X_rudder(4, :); X_rudder(10, :)]); ...
    zeros(size(X_rudder(1, :))); zeros(size(X_rudder(1, :))); ...
    X_rudder(5, :)];
% Plot results
plotData(X_elevatorPlot, time)

