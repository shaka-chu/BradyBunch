% AERO3560 - Flight Mechanics 1 - Assignment 1 2018
% Author SID: 460369684
% Function Name: euler2quat
%
% Function Description:
% Returns the quaternion components for a given Euler angle input matrix
%
% Inputs:
%   eulAngles: - matrix containing Euler angles (radians). The angles are
%                stored in order: [phi; theta; psi] = [roll; pitch; yaw]
%
% Outputs:
%   quaternion: - matrix of quaternion components [q0; q1; q2; q3]

function quaternion = euler2quat(eulAngles)

    % Unpack Euler angles matrix
    phi = eulAngles(1,:);
    theta = eulAngles(2,:);
    psi = eulAngles(3,:);
    
    % Divide by 2
    phi = phi/2;
    theta = theta/2;
    psi = psi/2;
    
    % Calculate quaternion components
    q0 = cos(psi).*cos(theta).*cos(phi) + sin(psi).*sin(theta).*sin(phi);
    q1 = cos(psi).*cos(theta).*sin(phi) - sin(psi).*sin(theta).*cos(phi);
    q2 = cos(psi).*sin(theta).*cos(phi) + sin(psi).*cos(theta).*sin(phi);
    q3 = -cos(psi).*sin(theta).*sin(phi) + sin(psi).*cos(theta).*cos(phi);
    
    % Return quaternion vector
    quaternion = [q0; q1; q2; q3];
    
end