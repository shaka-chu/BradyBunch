% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460369684
% Function Name: rotate321quat
%
% Function Description:
% Creates the DCM for a 3-2-1 series of axis rotations, using the
% quaternion formulation. Requires the quaternion that represents the
% rotation as an input.
%
% Inputs:
%   quaternion: column vector containing quaternions. They are stored in 
%               the order: [q0; q1; q2; q3]
%
% Outputs:
%   dcm_321: DCM for a standard 3-2-1 series of axis rotations

function dcm_321 = rotate321quat(quaternion)
    
    % Unpack quaternion matrix
    q0 = quaternion(1,:);
    q1 = quaternion(2,:);
    q2 = quaternion(3,:);
    q3 = quaternion(4,:);
    
    % Components of DCM
    c_11 = q3^2 + q0^2 - q1^2 -q2^2;
    c_12 = 2*(q0*q1 + q3*q2);
    c_13 = 2*(q0*q2 - q3*q1);
    c_21 = 2*(q0*q1 - q3*q2);
    c_22 = q3^2 - q0^2 + q1^2 - q2^2;
    c_23 = 2*(q1*q2 + q3*q0);
    c_31 = 2*(q0*q2 + q3*q1);
    c_32 = 2*(q1*q2 - q3*q0);
    c_33 = q3^2 - q0^2 - q1^2 + q2^2;
    
    % Formulate DCM
    dcm_321 = [c_11 c_12 c_13;
                c_21 c_22 c_23;
                c_31 c_32 c_33];
end