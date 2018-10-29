function validationCase

    % Lift curve slope
    a0 = 2*pi;        

    % Zero lift AoA (rad)
    alpha_0L_1 = -deg2rad(1.2);         % NACA 65-210
    
%     % Note: currently following TB example, which doesn't vary the airfoil
%     % over the wing span
%     alpha_0L_2 = 0;                     % NACA0012

    % Aircraft AoA (rad)
    alpha = deg2rad(4);

    % Taper ratio
    lambda = 0.4;

    % Aspect ratio
    AR = 9;

    % Control points (in polar coordinates)
    theta = deg2rad(linspace(0,90,5));
    theta = theta(2:end);                   % Remove 0 degrees

    % Number of control points
    nPoints = length(theta);

    % Generate vector of odd numbers (symmetrical wing)
    n = 2*(1:nPoints) - 1;

    % Preallocate for speed
    alphaCoefs  = NaN(nPoints,1);
    Amat        = NaN(nPoints);

    % Loop through control points
    for i = 1:nPoints

        % Zero-lift AoA changes with station
        if theta(i) <= pi/4
            alpha_0L = alpha_0L_1;
        else 
            alpha_0L = alpha_0L_1;
        end

        % Intermediate parameter
        mu = (a0/(2*(lambda + 1)*AR))*(1 + (lambda - 1)*cos(theta(i)));

        % Loop through Fourier coefficients
        for j = 1:nPoints

            % Current Fourier coefficient
            Amat(i,j) = sin(n(j)*theta(i))*(sin(theta(i)) + n(j)*mu);
        end

        % AoA coefficient
        alphaCoefs(i) = mu*sin(theta(i))*(alpha - alpha_0L);    
    end

    % Solve system of equations
    Avec = Amat\alphaCoefs;

    % Extract A1
    A1 = Avec(1);

    % Lift coefficient
    CL = pi*AR*A1;

    % Initialise delta
    delta = 0;

    % Parameter used for drag calculation
    for k = 2:length(n)
       delta = delta + n(k)*(Avec(k)^2/A1^2);
    end

    % Calculate induced drag
    Cdv = ((CL^2)/(pi*AR))*(1 + delta);
    
    % Print results to command window
    fprintf('---------------- Validation Case ---------------')
    fprintf('\n')
    fprintf('Example 3D Lift, CL: %.4g\n',CL)
    fprintf('Examlpe Induced Drag, CDi: %.4g\n',Cdv)
    fprintf('\n')

end