function [Ct, TailAngles, TailProps, ClimbTail] = liftingLineTail(n, ...
    nPts, U, alpha, Alpha0, A0, WingProps, TailProps, WingAngles)   

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
    fuselage_end_pos    = TailProps.FuseEnd;
    c_tip_t             = TailProps.TipChord;
    AR_t                = TailProps.AR;
    
    % Polar coordinates corresponding to wing features (rad)
    theta_fuselage = acos(fuselage_end_pos/s_t);

    % Control points for tailplane (in polar coordinates) (rad)
    theta_t = deg2rad(linspace(0,90,nPts + 1));
    theta_t = theta_t(2:end);                           % Remove 0 degrees

    % Convert wing/tailplane polar coords to spanwise positions
    y_wing      = s_w*cos(theta_w);
    y_wing(end) = 0;
    y_tail      = s_t*cos(theta_t);
    y_tail(end) = 0;
    
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

    % Loop through angles of attack
    for k = 1:length(alpha)
        
        % Loop through control points
        for i = 1:nPts

            % Properties change with station
            if theta_t(i) >= 0 && theta_t(i) < theta_fuselage

                % Lift curve slope for NACA 65-415
                a0 = a0_65415;

                % Zero-lift AoA for NACA 65-415
                alpha0 = alpha0_65415;

            elseif theta_t(i) > theta_fuselage && theta_t(i) <= pi/2

                % Lift curve slope for NACA 0012
                a0 = a0_0012;

                % Zero-lift AoA for NACA 0012
                alpha0 = alpha0_0012;  

            else
                warning('Theta_t outside bounds 0 <= theta_t <= pi/2');
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
            
            effAlpha(i) = alpha(k) - alpha_i_t(k,i);
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
    end
    
    % Struct for tailplane coefficients
    Ct.CL   = CL;
    Ct.Cdi  = Cdi;
    
    % Store wing chord in wing geometry struct
    TailProps.Chord = c;
    
    % Store angles (station positions and downwash) in struct
    TailAngles.Stations = theta_w;
    TailAngles.Downwash = alpha_i_t;
    
%     % Store cartesian station positions
%     CartStn.Wing = y_wing;
%     CartStn.Tail = y_tail;
    
    % Struct for climb properties (only used for climb)
    ClimbTail.Gamma             = gamma;
    ClimbTail.EffectiveAlpha    = effAlpha;
    ClimbTail.SpanPos           = y_tail;
end