% AERO3560 Assignment 4

% Make CSV
lateral = [EigAnalysis.DutchRoll.Pole, EigAnalysis.DutchRoll.Damping, EigAnalysis.DutchRoll.Wn, 1/EigAnalysis.DutchRoll.Wn;
           EigAnalysis.Spiral.Pole, EigAnalysis.Spiral.Damping, EigAnalysis.Spiral.Wn, 1/EigAnalysis.Spiral.Wn;
           EigAnalysis.Roll.Pole, EigAnalysis.Roll.Damping, EigAnalysis.Roll.Wn, 1/EigAnalysis.Roll.Wn];
       
longitudinal = [EigAnalysis.ShortPeriod.Pole, EigAnalysis.ShortPeriod.Damping, EigAnalysis.ShortPeriod.Wn, 1/EigAnalysis.ShortPeriod.Wn;
           EigAnalysis.Phugoid.Pole, EigAnalysis.Phugoid.Damping, EigAnalysis.Phugoid.Wn, 1/EigAnalysis.Phugoid.Wn;
           EigAnalysis.AltitudeConDi.Pole, EigAnalysis.AltitudeConDi.Damping, EigAnalysis.AltitudeConDi.Wn, 1/EigAnalysis.AltitudeConDi.Wn];

csvwrite('lateral.txt',lateral)
csvwrite('longitudinal.txt',longitudinal)
