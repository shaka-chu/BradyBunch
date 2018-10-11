% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: initialisation
%
% Function Description:
% Reads aircraft data from provided function
%
% Inputs:
%   This funciton does not take inputs
%
% Outputs:
%   nominal: Aircraft parameters at nominal CG position
%   secondary: Aircraft parameters at secondary CG position

function [Nominal_params, Secondary_params] = initialisation

    % Load data at nominal location
    Nominal_params = aero3560_LoadFlightDataPC9_nominalCG1();
    
    % Load data at secondary location
    Secondary_params = aero3560_LoadFlightDataPC9_CG2();
end