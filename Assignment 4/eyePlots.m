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
    
end