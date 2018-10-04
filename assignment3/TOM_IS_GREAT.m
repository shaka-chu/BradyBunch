% Test

Params = initialisation;

% load a bunch of shit
load ICs_PC9_CG2_100Kn_1000ft

CG2_100Kn_X = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
CG2_100Kn_U = U0;

load ICs_PC9_CG2_180Kn_1000ft

CG2_180Kn_X = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
CG2_180Kn_U = U0;

load ICs_PC9_nominalCG1_100Kn_1000ft

CG1_100Kn_X = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
CG1_100Kn_U = U0;

load ICs_PC9_nominalCG1_180Kn_1000ft

CG1_180Kn_X = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];
CG1_180Kn_U = U0;

isU0Right1 = trim(Params, CG2_100Kn_X);
isU0Right2 = trim(Params, CG2_180Kn_X);
isU0Right3 = trim(Params, CG1_100Kn_X);
isU0Right4 = trim(Params, CG1_180Kn_X);