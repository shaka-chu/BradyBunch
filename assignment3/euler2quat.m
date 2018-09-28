% AERO3560 - Flight Mechanics 1 - Assignment 1 2018
% Author SID: 460369684
% With Special Thanks To: 460306678
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
    
    % Calculate quaternion components
    q0 = cos(psi/2).*cos(theta/2).*cos(phi/2) + ...
        sin(psi/2).*sin(theta/2).*sin(phi/2);
    q1 = cos(psi/2).*cos(theta/2).*sin(phi/2) - ...
        sin(psi/2).*sin(theta/2).*cos(phi/2);
    q2 = cos(psi/2).*sin(theta/2).*cos(phi/2) + ...
        sin(psi/2).*cos(theta/2).*sin(phi/2);
    q3 = -cos(psi/2).*sin(theta/2).*sin(phi/2) + ...
        sin(psi/2).*cos(theta/2).*cos(phi/2);
    
    % Return quaternion vector
    quaternion = [q0; q1; q2; q3];
    
end