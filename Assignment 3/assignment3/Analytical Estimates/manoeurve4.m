% Roll control Calculations
% Author: 460306678


function nz=manoeurve4(X,time)
% Initialise Parameters
% Initialise aircraft parameters
[Nominal_params, Secondary_params] = initialisation;

% Select CG position
Params = Nominal_params;

% Extract necessary components
g = Params.Inertial.g;
mass = Params.Inertial.m;
S = Params.Geo.S;
c = Params.Geo.c;

% Set initial heading
phi_0 = 0;
psi_0 = 0;
theta_0 = 0;
euler = [phi_0; theta_0; psi_0];
quaternion_0 = euler2quat(euler);

% Set flight conditions
V = 120;
h = convlength(5000, 'ft','m');

% Create initial state
X0 = [V; 0; 0; 0 ; 0; 0; quaternion_0; 0; 0; -h];

% Trim aircraft
[X_trimmed, ~] = trim(Params, X0);

% Calculate atmospheric conditions
[rho, ~] = flowproperties(X_trimmed, V);

% Calculate equivalent airspeed
EAS = V*sqrt(rho/1.2256);
KEAS = convvel(EAS, 'm/s','kts');

% Calculate g loading
V = sqrt(X(1,:).^2 + X(2,:).^2 + X(3,:).^2);
nz = V.*X(5,:)/g + 1;

x = 0:length(time)/100;
y = 3.5*ones(1,length(x));

figure(7)
plot(time,nz)
hold on
plot(x,y,'lineWidth',1)
xticks([0:2:length(time)/100])
xlabel('Time (s)')
ylabel('N (g)')
grid on

% Find last attitude
quaternion = X(7:10,end);
euler = rad2deg(quat2euler(quaternion));
disp(euler(2));
height = abs(X(13,end)) - abs(X(13,1));
disp(height)




end