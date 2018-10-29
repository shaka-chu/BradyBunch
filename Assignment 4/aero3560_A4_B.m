% Clear command window and reset figure properties
clear;
clc;
clf;
clf reset;
close all;

% Load .mat file for CG2 at 220 kts
load A_lon_220Kn_500ft_CG2.mat

% Load aircraft properties
Params = aero3560_LoadFlightDataPC9_CG2;

% Flight speed
V = convvel(220,'kts','m/s');

% Density at flight speed
h               = convlength(500,'ft','m');
[~, ~, ~, rho]  = atmosisa(h);

% Pitch angle at steady level flight (radians)
theta = 0;

[Alat, Blat] = lateralStateSpace(Params, X)