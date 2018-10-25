function [Ct, TailAngles, TailProps, BankTail] = liftingLineTailBank(...
    nPts, alpha, U, Alpha0, A0, WingProps, TailProps, WingAngles)   

    % Unpack lift curve slopes (rad^-1)
    a0_0012     = A0.naca0012 ;             % NACA 0012
    a0_65415    = A0.naca65415;             % NACA 65-415

    % Unpack zero lift AoA (rad)
    alpha0_0012     = Alpha0.naca0012;      % NACA 0012
    alpha0_65415    = Alpha0.naca65415;     % NACA 65-415
    
    % Unpack wing angles
    theta_w     = WingAngles.Stations;
    alpha_i_w   = WingAngles.Downwash;

    % Unpack wing geometry
    s_w         = WingProps.SemiSpan;

    % Unpack tailplane geometry
    s_t                 = TailProps.SemiSpan;
    fuselage_end_pos_r  = TailProps.FuseEnd;
    fuselage_end_pos_l  = -fuselage_end_pos_r;
    c_tip_t             = TailProps.TipChord;
    AR_t                = TailProps.AR;
    
    % Polar coordinates corresponding to wing features (rad)
    theta_fuselage_r = acos(fuselage_end_pos_r/s_t);
    theta_fuselage_l = acos(fuselage_end_pos_l/s_t);
    
    % Vector of odd/even numbers (asymmetric loading)
    n = 1:nPts;

    % Control points for tailplane (in polar coordinates) (rad)
    theta_t = deg2rad(linspace(0,180,nPts + 2));
    theta_t = theta_t(2:end-1);                 % Remove 0 and 180 degrees

    % Convert wing/tailplane polar coords to spanwise positions
    y_wing      = s_w*cos(theta_w);
%     y_wing(end) = 0;
    y_tail      = s_t*cos(theta_t);
%     y_tail(end) = 0;
    
    % Interpolate downwash angles from wing on tailplane
    alpha_i_t = NaN(length(alpha),nPts);
    for k = 1:length(alpha)
        alpha_i_t(k,:) = interp1(y_wing,alpha_i_w(k,:),y_tail);
    end

    % Preallocate for speed
    alphaCoefs  = NaN(nPts,length(alpha));
    Amat        = NaN(nPts,nPts,length(alpha));
    c           = c_tip_t*ones(1,nPts);
    Avec        = zeros(nPts,length(alpha));
    CL          = zeros(1,length(alpha));
    Cdi         = zeros(1,length(alpha));
    effAlpha    = NaN(1,nPts);
    
    % Only one angle of attack used
    k = 1;
        
    % Loop through control points
    for i = 1:nPts

        % Tip to fuselage RIGHT
        if theta_t(i) >= 0 && theta_t(i) < theta_fuselage_r

            % Lift curve slope for NACA 65-415
            a0 = a0_65415;

            % Zero-lift AoA for NACA 65-415
            alpha0 = alpha0_65415;

        % Fuselage RIGHT -> LEFT
        elseif theta_t(i) > theta_fuselage_r && theta_t(i) <= theta_fuselage_l

            % Lift curve slope for NACA 0012
            a0 = a0_0012;

            % Zero-lift AoA for NACA 0012
            alpha0 = alpha0_0012;
            
        % Fuselage to tip LEFT    
        elseif theta_t(i) > theta_fuselage_l && theta_t(i) <= pi
            
            % Lift curve slope for NACA 65-415
            a0 = a0_65415;

            % Zero-lift AoA for NACA 65-415
            alpha0 = alpha0_65415;

        else
            warning('Theta_t outside bounds 0 <= theta_t <= pi');
        end

        % Intermediate parameter
        mu_t = (a0*c(i))/(8*s_t);

        % Loop through Fourier coefficients
        for j = 1:nPts

            % Current Fourier coefficient
            Amat(i,j,k) = sin(n(j)*theta_t(i))*(sin(theta_t(i)) + n(j)*mu_t);
        end

        % AoA coefficient
        alphaCoefs(i,k) = mu_t*sin(theta_t(i))*(alpha(k) - alpha0 - alpha_i_t(k,i));
        
        effAlpha(i) = alpha(k);
        
    end

    % Solve system of equations
    Avec(:,k) = Amat(:,:,k)\alphaCoefs(:,k);

    % Circulation
    for z = 1:length(theta_t)
        sum = 0;
        for d = 1:length(theta_t)
            sum = sum + Avec(d,k)*sin(n(d)*theta_t(z));
        end
        gamma(k,z) = 4*s_t*U*sum;
    end

    % Extract A1
    A1_t = Avec(1,k);

    % Tailplane lift coefficient
    CL(k) = pi*AR_t*A1_t;

    % Initialise delta
    delta = 0;

    % Parameter used for drag calculation
    for p = 2:length(n)
       delta = delta + n(p)*(Avec(p,k)^2/A1_t^2);
    end

    % Calculate induced drag
    Cdi(k) = ((CL(k)^2)/(pi*AR_t))*(1 + delta);
    
    % Struct for tailplane coefficients
    Ct.CL   = CL;
    Ct.Cdi  = Cdi;
    
    % Store wing chord in wing geometry struct
    TailProps.Chord = c;
    
    % Store angles (station positions and downwash) in struct
    TailAngles.Stations = theta_t;
    TailAngles.Downwash = alpha_i_t;
    
    % Struct for bank properties
    BankTail.Gamma             = gamma;
    BankTail.EffectiveAlpha    = effAlpha;
    BankTail.SpanPos           = y_tail;

end