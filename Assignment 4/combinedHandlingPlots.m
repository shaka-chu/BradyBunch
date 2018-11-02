function combinedHandlingPlots(nz_alpha)
    
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

    % Load data
    cruise1     = load('cruise1');
    cruise2     = load('cruise2');
    approach1   = load('approach1');
    approach2   = load('approach2');

    % Vectors for eye plot grid boundaries
    vertVec = linspace(0, 10, 10);
    horizVec = linspace(0, 4, 10);
    oneVec  = ones(1,10);
    
    % Create eye plot figure
    figure;
    hold on
    axis([0 2.5 0 10])
    xlabel('Damping Ratio, \zeta');
    ylabel('Undamped Natural Frequency, \omega_{n} (rad/s)');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    grid on
    
    % Plot short period frequency against damping
    p1 = plot(cruise1.EigAnalysis.ShortPeriod.Damping, ...
        cruise1.EigAnalysis.ShortPeriod.Wn, 'x');
    p2 = plot(cruise2.EigAnalysis.ShortPeriod.Damping, ...
        cruise2.EigAnalysis.ShortPeriod.Wn, 'x');
    p3 = plot(approach1.EigAnalysis.ShortPeriod.Damping, ...
        approach1.EigAnalysis.ShortPeriod.Wn, 'x');
    p4 = plot(approach2.EigAnalysis.ShortPeriod.Damping, ...
        approach2.EigAnalysis.ShortPeriod.Wn, 'x');
    
    % Plot grids for L1, L2, L3 bounds
    plot(minDamp_lvl1*oneVec, vertVec, maxDamp_lvl1*oneVec, vertVec, ...
        minDamp_lvl2*oneVec, vertVec, maxDamp_lvl2*oneVec, vertVec, ...
        minDamp_lvl3*oneVec, vertVec);
    plot(horizVec, minFreq_lvl1*oneVec, horizVec, maxFreq_lvl1*oneVec, ...
        horizVec, minFreq_lvl23*oneVec, horizVec, maxFreq_lvl23*oneVec);
    
    % Eye plot legend
    hleg1 = legend([p1, p2, p3, p4], '220 kts, nominal CG', ...
        '220 kts secondary CG', '90 kts nominal CG', ...
        '220 kts secondary CG');
    set(hleg1, 'Location', 'Best');
    

    % Bh roll - Flight level 1 - Class III
    D_min_1 = [0.08, 0.08];
    D_min_2 = [0.02, 0.02];
    D_min_3 = [0.0, 0.0];
    Wn_min_A = [0.4, 0.4];
    Wn_min_B = [0.4, 0.4];
    Wn_min_C = [0.4, 0.4];
    D_min_A_y = [0, 1.8];
    D_min_B_y = [0, 1.8];
    D_min_C_y = [0, 1.8];
    Wn_min_A_x = [D_min_2(1), 1];
    Wn_min_B_x = [D_min_1(1), 1];
    Wn_min_C_x = [D_min_3(1), 1];
    
    % Create dutch roll figure
    figure;
    hold on
    xlabel('\zeta_d');
    ylabel('\omega_{d_n}');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    ylim = 0:1.8;
    grid on
    
    % Plot fight conditions
    p5 = plot(cruise1.EigAnalysis.DutchRoll.Damping,cruise1.EigAnalysis.DutchRoll.Wn, 'x');
    p6 = plot(cruise2.EigAnalysis.DutchRoll.Damping,cruise2.EigAnalysis.DutchRoll.Wn, 'x');
    p7 = plot(approach1.EigAnalysis.DutchRoll.Damping,approach1.EigAnalysis.DutchRoll.Wn, 'x');
    p8 = plot(approach2.EigAnalysis.DutchRoll.Damping,approach2.EigAnalysis.DutchRoll.Wn, 'x');
    
    % Create array of \zeta_d to sovle for \omega_d
    zeta = 0.0278:0.0001:1;
    Wn = 0.05./zeta;
    p9 = plot(zeta, Wn);
    
    % Damping min
    plot(D_min_1, D_min_A_y, D_min_2, D_min_B_y, D_min_3, D_min_C_y)
    
    % Frequency min
    plot(Wn_min_A_x, Wn_min_A)
%     legend('level 1 - \zeta_d','level 2 - \zeta_d','level 3 - \zeta_d'...
%     ,'level 1-3 - \omega_{d_n}');

    hleg2 = legend([p5, p6, p7, p8, p9],'220 kts, nominal CG', ...
        '220 kts secondary CG', '90 kts nominal CG', ...
        '220 kts secondary CG', '\omega_n \times \zeta');
    set(hleg2, 'Location', 'Best')
    
end