function [Cw, WingAngles, WingProps, BankWing] = liftingLineBankWing( ...
    nPts, alpha, Alpha0, A0, WingProps, U)

    % Unpack lift curve slopes (rad^-1)
    a0_0012     = A0.naca0012 ;             % NACA 0012
    a0_65415    = A0.naca65415;             % NACA 65-415

    % Unpack zero lift AoA (rad)
    alpha0_0012     = Alpha0.naca0012;      % NACA 0012
    alpha0_65415    = Alpha0.naca65415;     % NACA 65-415
    
    % Unpack wing geometry
    s                   = WingProps.SemiSpan;
    taper_end_pos_r     = WingProps.TaperEnd;
    fuselage_end_pos_r  = WingProps.FuseEnd;
    taper_end_pos_l     = -taper_end_pos_r;
    fuselage_end_pos_l  = -fuselage_end_pos_r;
    ail_end_r           = WingProps.AilEnd;
    ail_start_r         = WingProps.AilStrt;
    ail_length          = WingProps.AilLength;
    ail_end_l           = -ail_end_r;
    ail_start_l         = -ail_start_r;
    c_root              = WingProps.RootChord;
    c_tip               = WingProps.TipChord;
    AR                  = WingProps.AR;

    % Polar coordinates corresponding to wing features (rad)
    theta_taper_r     = acos(taper_end_pos_r/s);
    theta_fuselage_r  = acos(fuselage_end_pos_r/s);
    theta_taper_l     = acos(taper_end_pos_l/s);
    theta_fuselage_l  = acos(fuselage_end_pos_l/s);
    theta_ail_end_r   = acos(ail_end_r/s);
    theta_ail_start_r = acos(ail_start_r/s);
    theta_ail_end_l   = acos(ail_end_l/s);
    theta_ail_start_l = acos(ail_start_l/s);
    
    % Aileron deflections (degrees)
    deflec_down = 25;
    
    % Interior angles (rad)
    ang_down = deg2rad(180 - deflec_down);
    
    % Wing LE to aileron TE diagonal length
    diag_down = sqrt(c_tip^2 + ail_length^2 - 2*c_tip*ail_length*cos(ang_down));
    
    % Effective angle of attack with aileron deflection (rad)
    alpha_ail_down  = asin((sin(ang_down)/diag_down)*ail_length);
    alpha_ail_up    = -alpha_ail_down;
    
    % Vector of odd/even numbers (asymmetric loading)
    n = 1:nPts;

    % Linear chord model (right wing)
    m_r           = (c_root - c_tip)/(theta_fuselage_r - theta_taper_r);
    intercept_r   = c_tip - m_r*theta_taper_r;
    
    % Linear chord model (left wing)
    m_l           = (c_tip - c_root)/(theta_taper_l - theta_fuselage_l);
    intercept_l   = c_root - m_l*theta_fuselage_l;

    % Control points for wing (in polar coordinates) (rad)
    theta_w = deg2rad(linspace(0,180,nPts + 2));
    theta_w = theta_w(2:end - 1);               % Remove 0 and 180 degrees
    
    % Cartesian points (used for plotting the chord vs span)
    y_wing = s*cos(theta_w);

    % Twist angle bounds (rad)
    twist_root  = deg2rad(2);
    twist_tip   = deg2rad(1);

    % Twist angle vector (rad)
    twist_angle = linspace(twist_tip,twist_root,nPts + 1);
    twist_angle = twist_angle(2:end);

    % Preallocate for speed
    alphaCoefs      = NaN(nPts,length(alpha));
    Amat            = NaN(nPts,nPts,length(alpha));
    c               = zeros(1,nPts);
    alpha_i         = zeros(length(alpha),nPts);
    Avec            = zeros(nPts,length(alpha));
    CL              = zeros(1,length(alpha));
    Cdi             = zeros(1,length(alpha));     
    effAlpha        = NaN(1,nPts);
    
    % Only one angle of attack used
    k = 1;
   
    % Loop through control points
    for i = 1:nPts

        % Tip to angle bit RIGHT
        if theta_w(i) >= 0 && theta_w(i) < theta_taper_r

            % Lift curve slope for NACA 65-415
            a0 = a0_65415;

            % Zero-lift AoA for NACA 65-415
            alpha0 = alpha0_65415;    

            % Wing chord
            c(i) = c_tip;

        % Angle bit RIGHT
        elseif theta_w(i) >= theta_taper_r && theta_w(i) < theta_fuselage_r

            % Lift curve slope for NACA 65-415
            a0 = a0_65415;

            % Zero-lift AoA for NACA 65-415
            alpha0 = alpha0_65415;

            % Current wing chord
            c(i) = intercept_r + m_r*theta_w(i);

        % Fuselage RIGHT -> LEFT
        elseif theta_w(i) >= theta_fuselage_r && theta_w(i) <= theta_fuselage_l

            % Lift curve slope for NACA 0012
            a0 = a0_0012;

            % Zero-lift AoA for NACA 0012
            alpha0 = alpha0_0012;

            % Wing chord
            c(i) = c_root;

        % Angle bit LEFT
        elseif theta_w(i) > theta_fuselage_l && theta_w(i) <= theta_taper_l

            % Lift curve slope for NACA 65-415
            a0 = a0_65415;

            % Zero-lift AoA for NACA 65-415
            alpha0 = alpha0_65415;

            % Current wing chord
            c(i) = intercept_l + m_l*theta_w(i);

        % Angle bit to tip LEFT
        elseif theta_w(i) > theta_taper_l && theta_w(i) <= pi

            % Lift curve slope for NACA 65-415
            a0 = a0_65415;

            % Zero-lift AoA for NACA 65-415
            alpha0 = alpha0_65415;

            % Wing chord
            c(i) = c_tip;

        else
            warning('Theta_w outside bounds 0 <= theta_w <= pi');
        end

        % Intermediate parameter
        mu_w = (a0*c(i))/(8*s);

        % Loop through Fourier coefficients
        for j = 1:nPts

            % Current Fourier coefficient
            Amat(i,j,k) = sin(n(j)*theta_w(i))*(sin(theta_w(i)) + ...
                            n(j)*mu_w);

        end

        % Right aileron
        if theta_w(i) >= theta_ail_end_r && theta_w(i) <= theta_ail_start_r

            % Set right aileron (+25 degrees) (in rad)
            aoa = alpha_ail_down + alpha;
            
            effAlpha(i) = aoa;

        % Left aileron
        elseif theta_w(i) >= theta_ail_start_l && theta_w(i) <= theta_ail_end_l

            % Set left aileron (-25 degrees) (in rad)
            aoa = alpha_ail_up + alpha;
            
            effAlpha(i) = aoa;

        % Otherwise, pitch angle
        else
            aoa = alpha;
            effAlpha(i) = aoa;
        end

        % AoA coefficient
        alphaCoefs(i,k) = mu_w*sin(theta_w(i))*(aoa - alpha0 + ...
                          twist_angle(i));
    end

    % Solve system of equations
    Avec(:,k) = Amat(:,:,k)\alphaCoefs(:,k);

    % Downwash angle at each station due to trailing vortices (rad)
    for z = 1:length(theta_w)

        % Initialise numerator
        num = 0;

        % Influence of trailing vortex of each station on current pos
        for d = 1:length(theta_w)
            num = num + n(d)*Avec(d,k)*sin(n(d)*theta_w(z));
        end

        % Downwash angle at 
        alpha_i(k,z) = num/sin(theta_w(z));

    end
    
    % Circulation
    for z = 1:length(theta_w)
        sum = 0;
        for d = 1:length(theta_w)
            sum = sum + Avec(d,k)*sin(n(d)*theta_w(z));
        end
        gamma(k,z) = 4*s*U*sum;
    end

    % Extract A1
    A1_w = Avec(1,k);

    % Lift coefficient
    CL(k) = pi*AR*A1_w;

    % Initialise delta
    delta = 0;

    % Parameter used for drag calculation
    for p = 2:length(n)
       delta = delta + n(p)*(Avec(p,k)^2/A1_w^2);
    end

    % Calculate induced drag
    Cdi(k) = ((CL(k)^2)/(pi*AR))*(1 + delta);

    % Struct for wing coefficients
    Cw.CL   = CL;
    Cw.Cdi  = Cdi;
    
    % Store wing chord in wing geometry struct
    WingProps.Chord = c;
    
    % Store angles (station positions and downwash) in struct
    WingAngles.Stations = theta_w;
    WingAngles.Downwash = alpha_i;
    
    % Struct for bank properties
    BankWing.Gamma             = gamma;
    BankWing.EffectiveAlpha    = effAlpha;
    BankWing.SpanPos           = y_wing;

end