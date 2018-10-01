% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460373315
% Function Name: bodyforces
%
% F_BODY = BODYFORCES(PARAMS, CL, CD, RHO, Q)
%
% Function Description:
% Returns the forces acting on the aircraft in the body axes
%
% Inputs:
%
% Outputs:
%   F_body = matrix of the combined body forces [F_bx; F_by; F_bz]

function F_body = bodyforces(Params, Cl, Cd, rho, Q, alpha, beta)
    
    % Calculate lift and drag forces from lift and drag coefficients 
    L = Cl*Q*Params.Geo.S;
    D = Cd*Q*Params.Geo.S;
    
    % Compute rotation from the aero angles into the body axes
    cz = rotatez(-beta);
    cy = rotatey(alpha);
    dcm_ba = cy*cz;
    
    
end