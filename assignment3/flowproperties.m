% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460369684
% Function Name: flowproperties
%
% [RHO, Q] = FLOWPROPERTIES(H,)
%
% Function Description:
% Returns the air density and dynamic pressure, calculated as a function of
% aircraft altitude and speed
%
% Inputs:
%   h:      Aircraft altitude (m)
%   V:      Magnitude of aircraft velocity in the flight direction
%
% Outputs:
%   rho:    Density of air at aircraft altitude (kg/m^3)
%   Q:      Dynamic pressure at aircraft altitude (Pa)

function [rho, Q] = flowproperties(h,V)

    % Constants
    T0  = 288.15;           % ISA SSL temperature (K)
    P0  = 101.325;          % ISA SSL pressure (kPa)
    R   = 0.287058;         % Ideal gas constant for air (kJ/kg.K)
  
    % Calculate atmospheric conditions based on altitude

    % Troposphere
    if h <= 11000
        
        % Temperature (K)
        temp = T0 - 0.0065*h;
        
        % Pressure (kPa)
        pressure = P0*(temp/T0)^5.256; 
    
    % Lower stratosphere
    elseif h <= 25000
        
        % Temperature (K)
        temp = -56.46 + 273.15;
        
        % Pressure (kPa)
        pressure = 22.65*exp(1.73 - 0.000157*h);
   
    % Upper stratosphere
    else
        
        % Stop simulation and display error message
        string = ['Altitude places aircraft in the upper stratosphere!',...
            ' Re-evaluate calculation of z in Earth axes!'];
        error(string);
    end

    % Calculate density (kg/m^3)
    rho = pressure/(R*temp);
    
    % Calculate dynamic pressure (Pa)
    Q = (1/2)*rho*V^2;
end