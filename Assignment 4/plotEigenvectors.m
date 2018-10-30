function plotEigenvectors(EigAnalysis, rowsLon, rowsLat)

    % Unpack eigenvectors
    phugVec = EigAnalysis.Phugoid.Eigenvector;
    shrtVec = EigAnalysis.ShortPeriod.Eigenvector;
    attiVec = EigAnalysis.AttitudeConDi.Eigenvector;
    dtchVec = EigAnalysis.DutchRoll.Eigenvector;
    sprlVec = EigAnalysis.Spiral.Eigenvector;
    rollVec = EigAnalysis.Roll.Eigenvector;
    
    % Labels on plots
    stateLon    = {'u','w','q','\theta','z_{e}'};
    stateLat    = {'v','p','r','\phi'};
    dx          = 0.02;
    dy          = 0.06;

    % Plot phugoid eigenvector
    phugEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Phugoid Mode Eigenvectors');
    xlabel('Re');
    ylabel('Im');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    grid on
    gridxy(0,0)
    for i = 1:rowsLon
        arrow3([0 0], [real(phugVec(i,1)) imag(phugVec(i,1))], 'k', ...
            0.9, 0.9);
        text(real(phugVec(i,1)) + dx, imag(phugVec(i,1)) + dy, ...
            stateLon{i});
    end
    
    % Plot short mode eigenvector
    shrtEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Short Mode Eigenvectors');
    xlabel('Re');
    ylabel('Im');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    grid on
    gridxy(0,0)
    for i = 1:rowsLon
        arrow3([0 0], [real(shrtVec(i,1)) imag(shrtVec(i,1))], 'k', ...
            0.9, 0.9);
        text(real(shrtVec(i,1)) + dx, imag(shrtVec(i,1)) + dy, ...
            stateLon{i});
    end
    
    % Plot attitude convergence/divergence mode eigenvector
    attiEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Attitude Convergence/Divergence Mode Eigenvectors');
    xlabel('Re');
    ylabel('Im');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    grid on
    gridxy(0,0)
    for i = 1:rowsLon
        arrow3([0 0], [real(attiVec(i,1)) imag(attiVec(i,1))], 'k', ...
            0.9, 0.9);
        text(real(attiVec(i,1)) + dx, imag(attiVec(i,1)) + dy, ...
            stateLon{i});
    end
    
    % Plot dutch roll mode eigenvector
    dtchEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Dutch Roll Mode Eigenvectors');
    xlabel('Re');
    ylabel('Im');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    grid on
    gridxy(0,0)
    for i = 1:rowsLat
        arrow3([0 0], [real(dtchVec(i,1)) imag(dtchVec(i,1))], 'k', ...
            0.9, 0.9);
        text(real(dtchVec(i,1)) + dx, imag(dtchVec(i,1)) + dy, ...
            stateLat{i});
    end
    
    % Plot spiral mode eigenvector
    sprlEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Spiral Mode Eigenvectors');
    xlabel('Re');
    ylabel('Im');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    grid on
    gridxy(0,0)
    for i = 1:rowsLat
        arrow3([0 0], [real(sprlVec(i,1)) imag(sprlVec(i,1))], 'k', ...
            0.9, 0.9);
        text(real(sprlVec(i,1)) + dx, imag(sprlVec(i,1)) + dy, ...
            stateLat{i});
    end
    
    % Plot roll mode eigenvector
    rollEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Roll Mode Eigenvectors');
    xlabel('Re');
    ylabel('Im');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    grid on
    gridxy(0,0)
    for i = 1:rowsLat
        arrow3([0 0], [real(rollVec(i,1)) imag(rollVec(i,1))], 'k', ...
            0.9, 0.9);
        text(real(rollVec(i,1)) + dx, imag(rollVec(i,1)) + dy, ...
            stateLat{i});
    end
end