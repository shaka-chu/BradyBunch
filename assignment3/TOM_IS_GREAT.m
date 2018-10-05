% Test

[Nominal_params, Secondary_params] = initialisation;
Params = Nominal_params;

% % load a bunch of shit
% load ICs_PC9_CG2_100Kn_1000ft
% 
% CG2_100Kn_X = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
% CG2_100Kn_U = U0;
% 
% load ICs_PC9_CG2_180Kn_1000ft
% 
% CG2_180Kn_X = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
% CG2_180Kn_U = U0;
% 
% load ICs_PC9_nominalCG1_100Kn_1000ft
% 
% CG1_100Kn_X = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
% CG1_100Kn_U = U0;

load ICs_PC9_nominalCG1_180Kn_1000ft

CG1_180Kn_X = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
CG1_180Kn_U = U0;

% Initial guess
alpha_dot_old = 0;
beta_dot_old = 0;

% Errors
error = 1;
tolerance = 1e-3;

% Initial state
X_old = CG1_180Kn_X;

% Set iteration limit
iterLim = 100;
iterCount = 0;

while error > tolerance
    
    angle_rates = [alpha_dot_old beta_dot_old];

    Xdot = staterates(Params, CG1_180Kn_X, CG1_180Kn_U, angle_rates);
    
    [alpha_dot, beta_dot] = alphabeta_dot(Xdot,CG1_180Kn_X);
    
    error_alpha_dot = abs((alpha_dot - alpha_dot_old)/alpha_dot_old);
    error_beta_dot = abs((beta_dot - beta_dot_old)/beta_dot_old);
    
    error = max([error_alpha_dot error_beta_dot]);
    
    
    alpha_dot_old = alpha_dot;
    beta_dot_old = beta_dot;
    
    
    
    
    if iterCount > iterLim
        warning('Reached iteration limit');
        break
    end
    
    iterCount = iterCount + 1;
 
end

% isU0Right1 = trim(Params, CG2_100Kn_X);
% isU0Right2 = trim(Params, CG2_180Kn_X);
% isU0Right3 = trim(Params, CG1_100Kn_X);
% isU0Right4 = trim(Params, CG1_180Kn_X);