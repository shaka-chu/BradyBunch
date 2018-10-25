function [Model, AoA, U] = loadExperiment

    % Load piper warrior model experimental data
    ExpData = readtable('piperData.xlsx');

    % Load piper warrior model support drag
    SupportData = readtable('supportDrag.xlsx');
    
    % Piper warrior model parameters - SOME APPROXIMATIONS (m)
    Model.SemiSpan          = 39.5/100;
    Model.FuselageWidth     = 11/100;
    Model.WingChord         = 17/100;
    Model.WingRootChord     = (1.86/1.6)*Model.WingChord;
    Model.TailChord         = 8/100;
    Model.TailSemiSpan      = Model.SemiSpan*0.370833333333333;
    Model.AircraftLength    = 67.5/100;
    Model.FuseStart         = Model.SemiSpan*0.105924259467567;
    Model.TaperEnd          = Model.SemiSpan*0.253655793025872;
    Model.TaperLength       = Model.SemiSpan*0.147731533558305;
    Model.WingArea          = 2*((Model.WingRootChord*Model.FuseStart) ...
                              + (Model.WingChord*Model.TaperLength) + ...
                              (1/2)*(Model.WingRootChord - ...
                              Model.WingChord)*Model.TaperLength + ...
                              Model.WingChord*(Model.SemiSpan - ...
                              Model.TaperEnd));
    Model.TailArea          = 2*Model.TailSemiSpan*Model.TailChord;
    Model.FuseStartTail     = Model.SemiSpan*0.0316647919010124;

    % Mean tunnel velocity (m/s)
    U = mean(ExpData.V);

    % Angles of attack 
    AoA.Degrees = ExpData.TrueAi;               % Deg
    AoA.Radians = deg2rad(ExpData.TrueAi);      % Rad

    % Wing area (m^2)
    Model.WingArea = Model.WingChord*(2*Model.SemiSpan + ...
                     Model.FuselageWidth);

    % Support reference area (m^2)
    supportArea = 0.75*0.15;

    % Obtain support drag coefficients at test tunnel speeds
    supportCd = interp1(SupportData.V,SupportData.Cds,ExpData.V, ...
                'nearest','extrap');
            
    % Interpolated drag
    supportDragInterp = supportCd.*(supportArea.*ExpData.DynP);
    
    % Support drag coefficient normalised by wing area
    supportCd_norm = supportDragInterp./(Model.WingArea.*ExpData.DynP);

    % Model lift coefficients
    Model.CL = ExpData.TrueL./(ExpData.DynP.*Model.WingArea);

    % Total drag coefficient of model and test stand
    totalCd = ExpData.TrueD./(ExpData.DynP.*Model.WingArea);

    % Model drag coefficients
    Model.Cd = totalCd - supportCd_norm;
    
    % Test dynamic pressures
    Model.DynP = ExpData.DynP;

end


