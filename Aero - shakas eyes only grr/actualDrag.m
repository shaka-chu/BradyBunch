function [CD0_actual, CLmax_actual] = actualDrag
    % Constant parameters
    rhoSL = 1.225;
    g = 9.81;

    % Piper Warrior II Parameters
    MTOW = 2440;            % Maximum gross weight  [lbs]
    Pmax = 160/1.341e-3;    % Maximum power         [W]
    AR = 7.2;               % Aspect ratio       
    dia = 74;               % Propeller diameter    [in]
    RPS = 2700/60;          % Propeller rotations per second

    % Wing area
    S = convlength(sqrt(170), 'ft', 'm')^2;

    % Aircraft weight force
    W = convmass(MTOW, 'lbm', 'kg')*g;

    % Estimate Oswald span efficienct factor from emperical formula
    e = 1/(1.05 + 0.007*pi*AR);
    k = 1/pi/AR/e;

    % Maximum airspeed in m/s
    Vfc = convvel(79, 'kts', 'm/s');

    % Best rate of climb in m/s
    RCmax = convvel(644, 'ft/min', 'm/s');

    % Propeller efficiency
    J = convvel(Vfc, 'mph', 'm/s')/RPS/convlength(dia, 'in', 'm');
    eta_p = 0.72;

    % Back calculate CD0 from performance data, fastest climb airspeed and
    % best rate of climb
    T = Pmax*eta_p/Vfc;
    D = T - RCmax*W/Vfc;
    CD = D/0.5/rhoSL/Vfc^2/S;
    CL = W/0.5/rhoSL/Vfc^2/S;
    CD0_actual = CD - k*CL^2;

    % Stall speed in m/s
    Vstall = convvel(56, 'kts', 'm/s');

    % Back calculate CLmax from performance data, stall speed
    CLmax_actual = 2*W/1.225/Vstall^2/S;
end