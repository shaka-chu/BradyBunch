function [Alat, Blat] = lateralStateSpace(Params, V, theta, h)

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
    
    % Density (kg/m^3)
    [~, ~, ~, rho] = atmosisa(h);
    
    % Dynamic pressure
    Q = (1/2)*rho*V^2;

    % Calculate aerodynamic derivatives for A matrix
    Q   = (1/2)*rho*V^2;
    Yv  = Q*S*Cyb/(m*V);
    Yp  = Q*S*b*Cyp/(2*m*V);
    Yr  = Q*S*b*Cyr/(2*m*V);
    Lv  = Q*S*b*Clb/(Ixx*V);
    Lp  = Q*S*b^2*Clp/(Ixx*2*V);
    Lr  = Q*S*b^2*Clr/(Ixx*2*V);
    Nv  = Q*S*b*Cnb/(Izz*V);
    Np  = Q*S*b^2*Cnp/(Izz*2*V);
    Nr  = Q*S*b^2*Cnr/(Izz*2*V);
    NTv = 0;
    A1  = Ixz/Ixx;
    B1  = Ixz/Izz;

    % Calculate elements of A matrix
    A11 = Yv;
    A12 = Yp;
    A13 = Yr - V;
    A14 = g*cos(theta);
    A15 = 0;
    A21 = (Lv + A1*(Nv + NTv))/(1 - A1*B1);
    A22 = (Lp + A1*Np)/(1 - A1*B1);
    A23 = (Lr + A1*Nr)/(1 - A1*B1);
    A24 = 0;
    A25 = 0;
    A31 = ((Nv + NTv) + B1*Lv)/(1 - A1*B1);
    A32 = (Np + B1*Lp)/(1 - A1*B1);
    A33 = (Nr + B1*Lr)/(1 - A1*B1);
    A34 = 0;
    A35 = 0;
    A41 = 0;
    A42 = 1;
    A43 = tan(theta);
    A44 = 0;
    A45 = 0;
    A51 = 0;
    A52 = 0;
    A53 = sec(theta);
    A54 = 0;
    A55 = 0;

    % Create lateral A matrix
    Alat = [A11 A12 A13 A14 A15;
            A21 A22 A23 A24 A25;
            A31 A32 A33 A34 A35;
            A41 A42 A43 A44 A45;
            A51 A52 A53 A54 A55];  

    % Calculate aerodynamic derivatives for B matrix
    Yda = (Q*S*Cyda)/m;
    Ydr = (Q*S*Cydr)/m;
    Lda = (Q*S*b*Clda)/Ixx;
    Ldr = (Q*S*b*Cldr)/Ixx;
    Nda = (Q*S*b*Cnda)/Izz;
    Ndr = (Q*S*b*Cndr)/Izz;

    % Calculate elements of B matrix
    B11 = Yda;
    B12 = Ydr;
    B21 = (Lda + A1*Nda)/(1 - A1*B1);
    B22 = (Ldr + A1*Ndr)/(1 - A1*B1);
    B31 = (Nda + B1*Lda)/(1 - A1*B1);
    B32 = (Ndr + B1*Ldr)/(1 - A1*B1);
    B41 = 0;
    B42 = 0;
    B51 = 0;
    B52 = 0;

    % Calculate lateral B matrix
    Blat = [B11 B12;
            B21 B22;
            B31 B32;
            B41 B42;
            B51 B52];
end