function Cdmin = dragBuildUp(U, WingProps, TailProps, FuseProps)

    % Unpack wing geometry
    S_ref       = WingProps.WingArea;
    c_root_w    = WingProps.RootChord;
    c_tip_w     = WingProps.TipChord;
    s_w         = WingProps.SemiSpan;
    
    % Unpack tail geometry
    c_root_t    = TailProps.RootChord;
    c_tip_t     = TailProps.RootChord;
    S_t         = TailProps.TailArea;

    % Unpack fuselage geometry
    length_f    = FuseProps.Length;
    diameter_f  = FuseProps.Diameter;
    f           = length_f/diameter_f;

    % Air properties (sea-level ISA)
    rho          = 1.225;           % Density (Kg/m^3)
    mu           = 1.789e-05;       % Viscosity (Pa.s)

    % Reynolds numbers of wing, tailplane and fuselage
    Re.WingRoot   = rho*U*c_root_w/mu;
    Re.WingTip    = rho*U*c_tip_w/mu;
    Re.TailRoot   = rho*U*c_root_t/mu;
    Re.TailTip    = rho*U*c_tip_t/mu;
    Re.Fuselage   = rho*U*length_f/mu;
    
    % Load Xfoil simulation data for transition points
    TransPoints = transitionPoints;

    % Wing plate friction coefficient
    Cf_avg_w = wingPlateCoef(Re, TransPoints);
    
    % Tail plate friction coefficient
    Cf_avg_t = tailPlateCoef(Re, TransPoints);
    
    % Fuselage plate drag coefficient (assume laminar flow over fuselage)
    Cf_f = 1.328/sqrt(Re.Fuselage);
    
    % Max thickness/chord ratio for NACA 65 series (Airfoiltools)
    tc_max = 0.15;
    
    % Wing wetted area (m^2)
    S_wet_w = 2*((1.60*0.788) + (1/2)*(0.26*0.788) + (1.60*3.89) + ...
              (s_w - 0.565)*tc_max*c_tip_w);
          
    % Tailplane wetted area (m^2)
    S_wet_t = S_t - 2*0.1689*c_root_t;
    
    % Ratio of fuselage wetted area to reference area based off Beech
    % Sierra (similar dimensions and shape) 
    areaRatio_f = 2.27;
    
    % Wing-fuselage interference factors
    Rwf = csvread('Rwf.csv');
    Rwf_w = interp1(Rwf(:,1)*1e6,Rwf(:,2),Re.WingRoot,'nearest','extrap');
    Rwf_t = interp1(Rwf(:,1)*1e6,Rwf(:,2),Re.TailRoot,'nearest','extrap');
    Rwf_f = interp1(Rwf(:,1)*1e6,Rwf(:,2),Re.Fuselage,'nearest','extrap');

%     % Wing and tailplane form factors (Hoerner)
%     FF_w = 1 + 2*tc_max + 60*tc_max^4;
%     FF_t = FF_w;
    
    % Fuselage form factor (Hoerner)
    FF_f = 1 + 60/f^3 + f/400;

    % Wing and tailplane form factors (Schemensky)
    FF_w = 1 + 1.44*tc_max + 2*tc_max^2;
    FF_t = FF_w;

    % Lifting surface correction factors (Schemensky)
    Rls_w = 1.05;
    Rls_t = 1.05;

    % Zero-lift drag of wing
    Cdmin_w = Rwf_w*Rls_w*Cf_avg_w*FF_w*(S_wet_w/S_ref);
    
    % Zero-lift drag of tailplane/epennage
    Cdmin_t = Rwf_t*Rls_t*Cf_avg_t*FF_t*(S_wet_t/S_ref);
    
    % Zero-lift drag of fuselage
    Cdmin_f = Rwf_f*Cf_f*FF_f*areaRatio_f;
    
    % Zero-lift drag of canopy
    S_canopy = 0.5*0.8; % Frontal area of the canopy
    Cdmin_canopy = 0.1*S_canopy/S_ref;

    % Total zero-lift drag
    Cdmin = Cdmin_w + Cdmin_t + Cdmin_f + Cdmin_canopy;
    
end