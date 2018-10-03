% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460373315
% Function Name: bodyforces
%
% [F_BODY, M_BODY] = BODYFORCES(PARAMS, CFA_X, CFA_Z, CL, ...
%       Q, ALPHA, BETA, ALPHA_DOT, BETA_DOT, DELTA_A, DELTA_E, ...
%       DELTA_R, P_HAT, Q_HAT, R_HAT, PHI)
%
% Function Description:
% Returns the forces acting on the aircraft in the body axes
%
% Inputs:
%
% Outputs:
%   F_body : matrix of the combined body forces [F_bx; F_by; F_bz] 
%   M_body : matrix of the combined moments [M_bL; M_bM; M_bN]

function [F_body, M_body] = bodyforces(Params, X, U, Cfa_x, Cfa_z, CL, ...
    Q, alpha, beta, alpha_dot, beta_dot, V)

    % Extract required aerodynamic parameters
    Clb     = Params.Aero.Clb;
    Clbd    = Params.Aero.Clbd;
    Clp     = Params.Aero.Clp;
    Clr     = Params.Aero.Clr;
    Clda    = Params.Aero.Clda;
    Cldr    = Params.Aero.Cldr;
    Cnb     = Params.Aero.Cnb;
    Cnbd    = Params.Aero.Cnbd;
    Cnp     = Params.Aero.Cnp;
    Cnr     = Params.Aero.Cnr;
    Cnda    = Params.Aero.Cnda;
    Cndr    = Params.Aero.Cndr;
    Cyb     = Params.Aero.Cyb;
    Cybd    = Params.Aero.Cybd;
    Cyp     = Params.Aero.Cyp;
    Cyr     = Params.Aero.Cyr;
    Cyda    = Params.Aero.Cyda;
    Cydr    = Params.Aero.Cydr;
    Cmo     = Params.Aero.Cmo;
    Cma     = Params.Aero.Cma;
    Cmq     = Params.Aero.Cmq;
    Cmad    = Params.Aero.Cmad;
    Cmde    = Params.Aero.Cmde;

    % Unpack state vector
    p   = X(4);
    q   = X(5);
    r   = X(6);
    q0  = X(7);
    q1  = X(8);
    q2  = X(9);
    q3  = X(10);
    
    % Determine euler angles from quaternion orientation
    quaternion = [q0; q1; q2; q3];
    euler_angles = quat2euler(quaternion);
    phi = euler_angles(1);
    
    % Extract required geometric parameters
    S = Params.Geo.S;
    b = Params.Geo.b;
    c = Params.Geo.c;
    
    % Extract control inputs
    delta_e = U(2);
    delta_a = U(3);
    delta_r = U(4);

    % Non-dimensionalise angular rates
    p_hat = (p*c)/(2*V);
    q_hat = (q*c)/(2*V);
    r_hat = (r*c)/(2*V);
    
    % Compute rotation from the aero angles into the body axes
    cz = rotatez(-beta);
    cy = rotatey(alpha);
    dcm_ba = cy*cz;
    
    % Calculate aerodynamic forces of the aircraft from in the wind
    % axes
    Fa_z = Q*Cfa_z*S;
    Fa_x = Q*Cfa_x*S;
    
    % Calculate the side force coefficient
    Cy = Cyb*beta + Cybd*beta_dot + Cyp*p_hat + Cyr*r_hat +...
        Cyda*delta_a + Cydr*delta_r + CL*phi;
    
    % Calculate side force
    Fa_y = Q*Cy*S;
    
    % Calculate moment coefficients
    Cl = Clb*beta + Clp*p_hat + Clr*r_hat + Clbd*beta_dot ...
        + Clda*delta_a + Cldr*delta_r;
    Cm = Cmo + Cma*alpha + Cmq*q_hat + Cmad*alpha_dot + Cmde*delta_e;
    Cn = Cnb*beta + Cnp*p_hat + Cnr*r_hat + Cnbd*beta_dot ... 
        + Cnda*delta_a + Cndr*delta_r;
    
    % Calculate moments
    La = Q*Cl*S*b;
    Ma = Q*Cm*S*b;
    Na = Q*Cn*S*b;
    
    % Sum the forces in the body axes
    F_body = dcm_ba*[Fa_x; Fa_y; Fa_z];
    M_body = dcm_ba*[La; Ma; Na];
end