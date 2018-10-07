% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: deltaE
%
% Function Description:
%   Simulates the flight path on aircraft model
%
% Inputs:
%
%
% Other m-files required:
%   None
%
% Subfunctions:
%   None
%
% MAT-files required: none
%
% TODO: 
%   None

function de = deltaE(Params,X, U_trimmed, n)

% Extract necessary components
g = Params.Inertial.g;
mass = Params.Inertial.m;
S = Params.Geo.S;
c = Params.Geo.c;

% Calculate atmospheric conditions
V = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
[rho, qbar] = flowproperties(X, V);

% Static parameters
Cm_q = -7.8179;
dCmcg_dCl_x = - 0.0752;
Cbar_me_de = -0.8405;

% Calculate necessary deflections
de = U_trimmed(2) - ((mass*g/S)./qbar).*(1/Cbar_m_de)* ...
        ((dCmcg_dCl_x)*n + Cm_q * (cbar*rho*g)*(n-1)/(4*(mass*g/S)));



end