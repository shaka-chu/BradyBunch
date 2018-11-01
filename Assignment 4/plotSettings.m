function plotSettings

    % Plotting colors
    red     = [0.8471 0.1176 0.1922];
    blue    = [0.1059 0.3882 0.6157];
    black   = [0 0 0];
    green   = [0 0.5020 0];
    cyan    = [0.0078 0.6627 0.8863];
    yellow  = [0.9843 0.7608 0.0510];
    gray    = [0.3490 0.3490 0.3490];
    orange  = [1.0000 0.4980 0];
    purple  = [0.5961 0.3059 0.6392];

    % Plotting line width
    lw = 1.2;

    % Set plotting settings
    set(groot,'defaultAxesColorOrder',[black;blue;red;green;yellow;cyan;...
        purple;orange]);
    set(0,'defaultLineLineWidth',lw);
    
    % Font
    set(0, 'defaultAxesFontName','Latin Modern Math')
    set(0,'defaultTextFontName', 'Latin Modern Math')
    set(0, 'defaultAxesFontSize', 18)
    %set(0, 'defaultAxesFontWeight', 'bold')
    set(0, 'defaultTextInterpreter','tex')
    position = [10, 50, 1280, 720];
%     set(0, 'DefaultFigurePosition', position);

end