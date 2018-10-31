function plotRootLocus(EigAnalysis)

    % Unpack eigenvalues
    phugEigVal = EigAnalysis.Phugoid.Pole;
    shrtEigVal = EigAnalysis.ShortPeriod.Pole;
    attiEigVal = EigAnalysis.AttitudeConDi.Pole;
    dtchEigVal = EigAnalysis.DutchRoll.Pole;
    sprlEigVal = EigAnalysis.Spiral.Pole;
    rollEigVal = EigAnalysis.Roll.Pole;
    
    % Plot colours
    magenta = [1 0 1];
    green = [0 1 0];
    red = [1 0 0];
    

    % Plot longitudinal-directional root locus
    figure;
    hold on
    p1 = plot(real(phugEigVal), imag(phugEigVal), 'o', 'Color', ...
        magenta, 'MarkerSize', 10);
    p2 = plot(real(shrtEigVal), imag(shrtEigVal), 'o', 'Color', ...
        green, 'MarkerSize', 10);
    p3 = plot(real(attiEigVal), imag(attiEigVal), 'o', 'Color', ...
        red, 'MarkerSize', 10);
    xlabel('Re');
    ylabel('Im');
    grid on
    gridxy(0,0);
    hleg1 = legend([p1, p2, p3],'Phugoid Mode', 'Short Period Mode', ...
        'Attitude Convergence/Divergence Mode');
    set(hleg1, 'Location', 'Best');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
    
    % Plot lateral-directional root locus
    figure;
    hold on
    p4 = plot(real(dtchEigVal), imag(dtchEigVal), 'o', 'Color', ...
        magenta, 'MarkerSize', 10);
    p5 = plot(real(sprlEigVal), imag(sprlEigVal), 'o', 'Color', ...
        green, 'MarkerSize', 10);
    p6 = plot(real(rollEigVal), imag(rollEigVal), 'o', 'Color', ...
        red, 'MarkerSize', 10);
    xlabel('Re');
    ylabel('Im');
    grid on
    gridxy(0,0);
    hleg2 = legend([p4, p5, p6],'Dutch Roll Mode', 'Spiral Mode', ...
        'Roll Mode');
    set(hleg2, 'Location', 'Best');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    

end