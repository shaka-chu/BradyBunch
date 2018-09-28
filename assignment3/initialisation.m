% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678
% Function Name: initialisation
%
% Function Description:
% Reads aircraft data from provided function
%
% Inputs:
%   Input: ----------------------------------------------
%
% Outputs:
%   Output: - Aicraft parameters at two different centre of gravity
%   location

function [nominal, secondary] = initialisation()

    % Load data at nominal location
    nominal = aero3560_LoadFlightDataPC9_nominalCG1();
    
    % Load data at secondary location
    secondary = aero3560_LoadFlightDataPC9_CG2();


end