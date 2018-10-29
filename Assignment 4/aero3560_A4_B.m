% Clear command window and reset figure properties
clear;
clc;
clf;
clf reset;
close all;

% Flight condition
airspeed    = 'cruise';
cgPos       = '2';
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

