% AERO3560 Assignment 4
% Author: 460306678

function [] = handlingQualities(Params, V, h, EigAnalysis)

    % Unpack intertial data
    g   = Params.Inertial.g;
    m   = Params.Inertial.m;
    Ixx = Params.Inertial.Ixx;
    Izz = Params.Inertial.Izz;
    Ixz = Params.Inertial.Ixz;

    % Unpack geometric data
    S   = Params.Geo.S;
    b   = Params.Geo.b;

    % Unpack side force coefficients
    Cyb = Params.Aero.Cyb;
    Cyp = Params.Aero.Cyp;
    Cyr = Params.Aero.Cyr;
    Cyda = Params.Aero.Cyda;
    Cydr = Params.Aero.Cydr;

    % N Moment Coefficients
    Cnb = Params.Aero.Cnb;
    Cnp = Params.Aero.Cnp;
    Cnr = Params.Aero.Cnr;
    Cnda = Params.Aero.Cnda;
    Cndr = Params.Aero.Cndr;

    % L Moment Coefficients
    Clb = Params.Aero.Clb;
    Clp = Params.Aero.Clp;
    Clr = Params.Aero.Clr;
    Clda = Params.Aero.Clda;
    Cldr = Params.Aero.Cldr;
    
    % Extract necessary aircraft parameters
    CLo     = Params.Aero.CLo;          % Non-dimensional
    CLa     = Params.Aero.CLa;          % /rad
    CLq     = Params.Aero.CLq;          % Non-dimensional
    CLde    = Params.Aero.CLde;         % /rad
    CLad    = Params.Aero.CLad;         % /rad
    Cdo     = Params.Aero.Cdo;          % Non-dimensional
    k       = Params.Aero.k;            % Non-dimensional
    c       = Params.Geo.c;             % m
    
    % M Moment Coefficients
    Cmo = Params.Aero.Cmo;
    Cma = Params.Aero.Cma;
    Cmq = Params.Aero.Cmq;
    Cmad = Params.Aero.Cmad;
    Cmde = Params.Aero.Cmde;
    
    % Density (kg/m^3)
    [~, ~, ~, rho] = atmosisa(h);
    
    % Dynamic pressure
    Q = (1/2)*rho*V^2;    

    % Determing loading per angle of attack
    nz_alpha =(CLa - (CLde/Cmde)*Cma)/((m*g/(Q*S)) - (g*c/(2*V^2))*(CLq - (CLde/Cmde)*Cmq));
    
    % Eye plots
    eyePlots(EigAnalysis, nz_alpha);

end