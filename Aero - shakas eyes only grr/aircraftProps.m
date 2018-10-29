function [WingProps, TailProps, FuseProps] = aircraftProps

    % Wing geometry (real aircraft)
    WingProps.Span      = convlength(35,'ft','m');
    WingProps.SemiSpan  = WingProps.Span/2;
    WingProps.TaperEnd  = 1.353;
    WingProps.FuseEnd   = 0.565;
    WingProps.RootChord = 1.86;
    WingProps.TipChord  = 1.60;
    WingProps.WingArea  = 2*((1.86*0.565) + (1.60*0.788) + (1/2)* ...
                          (0.26*0.788) + (1.60*3.89));
    WingProps.AR        = (WingProps.Span^2)/(WingProps.WingArea);
    WingProps.AilStrt   = 7*((convlength(3,'ft','m') + ...
                          convlength(6.2,'in','m'))/3);
    WingProps.AilEnd    = 7.2*((convlength(3,'ft','m') + ...
                          convlength(6.2,'in','m'))/3) + WingProps.AilStrt;
    WingProps.AilLength = ((0.5 + 0.45)/2)*((convlength(3,'ft','m') + ...
                          convlength(6.2,'in','m'))/3);

    % Tailplane geometry
    TailProps.Span      = convlength(12,'ft','m') + ...
                          convlength(11.75,'in','m');
    TailProps.SemiSpan  = TailProps.Span/2;
    TailProps.FuseEnd   = 0.1689;
    TailProps.RootChord = 0.762;
    TailProps.TipChord  = 0.762;
    TailProps.TailArea  = (TailProps.RootChord*TailProps.Span);
    TailProps.AR        = (TailProps.Span^2)/(TailProps.TailArea);
    
    % Fuselage geometry
    FuseProps.Length = 7.25;
    FuseProps.Diameter = 1.14;

end