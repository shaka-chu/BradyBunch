function eyePlots(EigAnalysis, nz_alpha)

    % Category B damping ratios
    maxDamp_lvl1 = 2;
    minDamp_lvl1 = 0.3;
    maxDamp_lvl2 = 2;
    minDamp_lvl2 = 0.2;
    minDamp_lvl3 = 0.15;
    
    % Load category B frequency plots
    minPlot_lvl1    = csvread('lvl1_min.csv');
    maxPlot_lvl1    = csvread('lvl1_max.csv');
    minPlot_lvl23   = csvread('lvl23_min.csv');
    maxPlot_lvl23   = csvread('lvl23_max.csv');
    
    % Linearly interpolate frequency plots
    minFreq_lvl1    = interp1(minPlot_lvl1(:,1), minPlot_lvl1(:,2), ...
        nz_alpha);
    maxFreq_lvl1    = interp1(maxPlot_lvl1(:,1), maxPlot_lvl1(:,2), ...
        nz_alpha);
    minFreq_lvl23   = interp1(minPlot_lvl23(:,1), minPlot_lvl23(:,2), ...
        nz_alpha);
    maxFreq_lvl23   = interp1(maxPlot_lvl23(:,1), maxPlot_lvl23(:,2), ...
        nz_alpha);
    
    % Plot grids for L1, L2, L3 bounds
    vertVec = linspace(0, 10, 10);
    horizVec = linspace(0, 4, 10);
    oneVec  = ones(1,10);
    figure;
    hold on
    axis([0 2.5 0 10])
    xlabel('Damping Ratio, \zeta');
    ylabel('Undamped Natural Frequency, \omega_{n} (rad/s)');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    grid on
    plot(minDamp_lvl1*oneVec, vertVec, maxDamp_lvl1*oneVec, vertVec, ...
        minDamp_lvl2*oneVec, vertVec, maxDamp_lvl2*oneVec, vertVec, ...
        minDamp_lvl3*oneVec, vertVec);
    plot(horizVec, minFreq_lvl1*oneVec, horizVec, maxFreq_lvl1*oneVec, ...
        horizVec, minFreq_lvl23*oneVec, horizVec, maxFreq_lvl23*oneVec);
    plot(EigAnalysis.ShortPeriod.Damping, EigAnalysis.ShortPeriod.Wn, 'x')


    
    % Attempt at ellipse plotting
    
    % Middle points for L1
    midDampL1 = (maxDamp_lvl1 + minDamp_lvl1)/2;
    midFreqL1 = (maxFreq_lvl1 + minFreq_lvl1)/2;
    
    % L1 semi-major axis coordinates
    x1  = minDamp_lvl1;
    y1  = midFreqL1;
    x2  = maxDamp_lvl1;
    y2  = midFreqL1;
    aL1 = abs((x2 - x1)/2);
    
    % L1 semi-minor axis coordinates
    x3  = midDampL1;
    y3  = minFreq_lvl1;
    x4  = midDampL1;
    y4  = maxFreq_lvl1;
    bL1 = abs((y4 - y3)/2);
    
    % Ellipse center coordinates
    x5 = midDampL1;
    y5 = midFreqL1;
    
    % Parameterisation of ellipse
    M = linspace(0,2*pi);           % Mean anomaly (rad)
    xL1 = x5 + aL1*cos(M);
    yL1 = y5 + bL1*sin(M);
    plot(xL1,yL1,'k-');
    
    % Middle points for L2
    midDampL2 = (maxDamp_lvl2 + minDamp_lvl2)/2;
    midFreqL2 = (maxFreq_lvl23 + minFreq_lvl23)/2;
    
    % L2 semi-major axis coordinates
    x1  = minDamp_lvl2;
    y1  = midFreqL2;
    x2  = maxDamp_lvl2;
    y2  = midFreqL2;
    aL2 = abs((x2 - x1)/2);
    
    % L1 semi-minor axis coordinates
    x3  = midDampL2;
    y3  = minFreq_lvl23;
    x4  = midDampL2;
    y4  = maxFreq_lvl23;
    bL2 = abs((y4 - y3)/2);
    
    % Ellipse center coordinates
    x5 = midDampL2;
    y5 = midFreqL2;
    
    % Parameterisation of ellipse
    M = linspace(0,2*pi);           % Mean anomaly (rad)
    xL2 = x5 + aL2*cos(M);
    yL2 = y5 + bL2*sin(M);
    plot(xL2,yL2,'k-');
    
    
    
end