% Test

% load a bunch of shit
load ICs_PC9_CG2_100Kn_1000ft

CG2_100Kn_X = X0;
CG2_100Kn_U = U0;

load ICs_PC9_CG2_180Kn_1000ft

CG2_180Kn_X = X0;
CG2_180Kn_U = U0;

load ICs_PC9_nominalCG1_100Kn_1000ft

CG1_100Kn_X = X0;
CG1_100Kn_U = U0;

load ICs_PC9_nominalCG1_180Kn_1000ft

CG1_180Kn_X = X0;
CG1_180Kn_U = U0;

isU0Right1 = trim(CG2_100Kn_X);
isU0Right2 = trim(CG2_180Kn_X);
isU0Right3 = trim(CG1_100Kn_X);
isU0Right4 = trim(CG1_180Kn_X);