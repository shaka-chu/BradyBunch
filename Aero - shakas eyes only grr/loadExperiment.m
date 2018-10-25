function [Model, AoA, U] = loadExperiment

    % Load piper warrior model experimental data
    ExpData = readtable('piperData.xlsx');

    % Load piper warrior model support drag
    SupportData = readtable('supportDrag.xlsx');
    
    % Piper warrior model parameters (m)
    Model.Semispan          = 39.5/100;
    Model.FuselageWidth     = 11/100;
    Model.WingChord         = 17/100;
    Model.TailChord         = 17/100;
    Model.AircraftLength    = 67.5/100;

    % Mean tunnel velocity (m/s)
    U = mean(ExpData.V);

    % Angles of attack 
    AoA.Degrees = ExpData.TrueAi;               % Deg
    AoA.Radians = deg2rad(ExpData.TrueAi);      % Rad

    % Wing area (m^2) (VERY approximate - doesnt account for the taper bit!)
    Model.WingArea = Model.WingChord*(2*Model.Semispan + Model.FuselageWidth);

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


