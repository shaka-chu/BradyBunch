function [Cw, WingAngles, WingProps, ClimbWing] = liftingLineWing(n, ...
    nPts, alpha, Alpha0, A0, WingProps, U)

    % Unpack lift curve slopes (rad^-1)
    a0_0012     = A0.naca0012 ;             % NACA 0012
    a0_65415    = A0.naca65415;             % NACA 65-415

    % Unpack zero lift AoA (rad)
    alpha0_0012     = Alpha0.naca0012;      % NACA 0012
    alpha0_65415    = Alpha0.naca65415;     % NACA 65-415
    
    % Unpack wing geometry
    s                   = WingProps.SemiSpan;
    taper_end_pos       = WingProps.TaperEnd;
    fuselage_end_pos    = WingProps.FuseEnd;
    c_root              = WingProps.RootChord;
    c_tip               = WingProps.TipChord;
    AR                  = WingProps.AR;

    % Polar coordinates corresponding to wing features (rad)
    theta_taper     = acos(taper_end_pos/s);
    theta_fuselage  = acos(fuselage_end_pos/s);

    % Linear chord model
    m           = (c_root - c_tip)/(theta_fuselage - theta_taper);
    intercept   = c_tip - m*theta_taper;

    % Control points for wing (in polar coordinates) (rad)
    theta_w = deg2rad(linspace(0,90,nPts + 1));
    theta_w = theta_w(2:end);                       % Remove 0 degrees
    
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
    
    % Loop through angles of attack
    for k = 1:length(alpha)

        % Loop through control points
        for i = 1:nPts

            % Tip to angle bit
            if theta_w(i) >= 0 && theta_w(i) < theta_taper

                % Lift curve slope for NACA 65-415
                a0 = a0_65415;

                % Zero-lift AoA for NACA 65-415
                alpha0 = alpha0_65415;    

                % Wing chord
                c(i) = c_tip;

            % Angle bit
            elseif theta_w(i) >= theta_taper && theta_w(i) < theta_fuselage

                % Lift curve slope for NACA 65-415
                a0 = a0_65415;

                % Zero-lift AoA for NACA 65-415
                alpha0 = alpha0_65415;

                % Current wing chord
                c(i) = intercept + m*theta_w(i);

            % Fuselage
            elseif theta_w(i) >= theta_fuselage && theta_w(i) <= pi/2

                % Lift curve slope for NACA 0012
                a0 = a0_0012;

                % Zero-lift AoA for NACA 0012
                alpha0 = alpha0_0012;

                % Wing chord
                c(i) = c_root;

            else
                warning('Theta_w outside bounds 0 <= theta_w <= pi/2');
            end

            % Intermediate parameter
            mu_w = (a0*c(i))/(8*s);

            % Loop through Fourier coefficients
            for j = 1:nPts

                % Current Fourier coefficient
                Amat(i,j,k) = sin(n(j)*theta_w(i))*(sin(theta_w(i)) + ...
                                n(j)*mu_w);

            end

            % AoA coefficient
            alphaCoefs(i,k) = mu_w*sin(theta_w(i))*(alpha(k) - ...
                                alpha0 + twist_angle(i));
                            
                            
            % Effective angle of attack (excluding twist)
            effAlpha(i) = alpha(k);

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

    end
    
    % Struct for wing coefficients
    Cw.CL   = CL;
    Cw.Cdi  = Cdi;
    
    % Store wing chord in wing geometry struct
    WingProps.Chord = c;
    
    % Store angles (station positions and downwash) in struct
    WingAngles.Stations = theta_w;
    WingAngles.Downwash = alpha_i;
    
    % Struct for climb properties (only used for climb)
    ClimbWing.Gamma             = gamma;
    ClimbWing.EffectiveAlpha    = effAlpha;
    ClimbWing.SpanPos           = y_wing;
end