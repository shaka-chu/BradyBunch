function plotEigenvectors(EigAnalysis, rowsLon, rowsLat)

    % Unpack eigenvectors
    phugVec = EigAnalysis.Phugoid.Eigenvector;
    shrtVec = EigAnalysis.ShortPeriod.Eigenvector;
    attiVec = EigAnalysis.AttitudeConDi.Eigenvector;
    dtchVec = EigAnalysis.DutchRoll.Eigenvector;
    sprlVec = EigAnalysis.Spiral.Eigenvector;
    rollVec = EigAnalysis.Roll.Eigenvector;

    % Plot phugoid eigenvector
    phugEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Phugoid Mode Eigenvectors');
    xlabel('Re')
    ylabel('Im')
    grid on
    for i = 1:rowsLon
        arrow3([0 0], [real(phugVec(i,1)) imag(phugVec(i,1))], 'k', ...
            0.9, 0.9)
    end
    
    % Plot short mode eigenvector
    shrtEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Short Mode Eigenvectors');
    xlabel('Re')
    ylabel('Im')
    grid on
    for i = 1:rowsLon
        arrow3([0 0], [real(shrtVec(i,1)) imag(shrtVec(i,1))], 'k', ...
            0.9, 0.9)
    end
    
    % Plot attitude convergence/divergence mode eigenvector
    attiEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Attitude Convergence/Divergence Mode Eigenvectors');
    xlabel('Re')
    ylabel('Im')
    grid on
    for i = 1:rowsLon
        arrow3([0 0], [real(attiVec(i,1)) imag(attiVec(i,1))], 'k', ...
            0.9, 0.9)
    end
    
    % Plot dutch roll mode eigenvector
    dtchEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Dutch Roll Mode Eigenvectors');
    xlabel('Re')
    ylabel('Im')
    grid on
    for i = 1:rowsLat
        arrow3([0 0], [real(dtchVec(i,1)) imag(dtchVec(i,1))], 'k', ...
            0.9, 0.9)
    end
    
    % Plot spiral mode eigenvector
    sprlEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Spiral Mode Eigenvectors');
    xlabel('Re')
    ylabel('Im')
    grid on
    for i = 1:rowsLat
        arrow3([0 0], [real(sprlVec(i,1)) imag(sprlVec(i,1))], 'k', ...
            0.9, 0.9)
    end
    
    % Plot roll mode eigenvector
    rollEigVec = figure;
    hold on
    axis([-1 1 -1 1]);
    daspect([1 1 1]);
    title('Roll Mode Eigenvectors');
    xlabel('Re')
    ylabel('Im')
    grid on
    for i = 1:rowsLat
        arrow3([0 0], [real(rollVec(i,1)) imag(rollVec(i,1))], 'k', ...
            0.9, 0.9)
    end
end