function EigAnalysis = eigenAnalysis(Alon, Alat, printAnalysis, ...
    plsPlotEigenVecs)

    % Obtain eigenvectors for longitudinal state space
    [Ulon, eigMatLon] = eig(Alon);
    
    % Longitudinal eigenvalues corresponding to eigenvector matrix cols
    eigValsVecLon = diag(eigMatLon);
    
    % Number of longitudinal states
    [rowsLon, ~] = size(Alon);
    
    % Obtain eigenvectors for lateral state space
    [Ulat, eigMatLat] = eig(Alat(1:4,1:4));
    
    % Lateral eigenvalues corresponding to eigenvector matrix cols
    eigValsVecLat = diag(eigMatLat);
    
    % Number of lateral states
    [rowsLat, ~] = size(Alat(1:4,1:4));
    
    % Obtain frequency, damping and poles of longitudinal state space
    [wnLon, dampLon, polesLon] = damp(Alon);
    
    % Obtain frequency, damping and poles of longitudinal state space
    [wnLat, dampLat, polesLat] = damp(Alat(1:4,1:4));
    
    % Longitudinal-directional data in order of frequency
    [maxFreqLon, maxIndLon] = maxk(wnLon, rowsLon);
    
    % Store longitudinal-directional eigen analysis in struct
    EigAnalysis.ShortPeriod.Wn          = maxFreqLon(1);
    EigAnalysis.ShortPeriod.Damping     = dampLon(maxIndLon(1));
    EigAnalysis.ShortPeriod.Pole        = polesLon(maxIndLon(1));
    EigAnalysis.Phugoid.Wn              = maxFreqLon(3);
    EigAnalysis.Phugoid.Damping         = dampLon(maxIndLon(3));
    EigAnalysis.Phugoid.Pole            = polesLon(maxIndLon(3));
    EigAnalysis.AttitudeConDi.Wn        = maxFreqLon(5);
    EigAnalysis.AttitudeConDi.Damping   = dampLon(maxIndLon(5));
    EigAnalysis.AttitudeConDi.Pole      = polesLon(maxIndLon(5));
    
    % Find indexes of eigenvectors for longitudinal modes mode
    phugVecInd = find(eigValsVecLon == EigAnalysis.Phugoid.Pole);
    shrtVecInd = find(eigValsVecLon == EigAnalysis.ShortPeriod.Pole);
    attiVecInd = find(eigValsVecLon == EigAnalysis.AttitudeConDi.Pole);
    
    % Eigenvectors for longitudinal modes
    eigenVecPhugoid     = Ulon(:,phugVecInd(1));
    eigenVecShort       = Ulon(:,shrtVecInd(1));
    eigenVecAttitude    = Ulon(:,attiVecInd(1));
    
    % Store longitudinal-directional eigenvectors
    EigAnalysis.Phugoid.Eigenvector         = eigenVecPhugoid;
    EigAnalysis.ShortPeriod.Eigenvector     = eigenVecShort;
    EigAnalysis.AttitudeConDi.Eigenvector   = eigenVecAttitude;
    
    % Find complex pair index positions (dutch roll)
    dutchInd = find(imag(polesLat) ~= 0);
    
    % Find minimum frequency index (long-term convergence spiral)
    [~, spiralInd] = min(wnLat);
    
    % Find index that isnt Dutch roll or spiral (i.e. roll)
    indexes = 1:4;
    notRoll = [dutchInd; spiralInd];
    mem     = ismember([1 2 3 4], notRoll);
    rollInd = indexes(~mem);
    
    % Store lateral-directional eigen analysis in struct
    EigAnalysis.DutchRoll.Wn        = wnLat(dutchInd(1));
    EigAnalysis.DutchRoll.Damping   = dampLat(dutchInd(1));
    EigAnalysis.DutchRoll.Pole      = polesLat(dutchInd(1));
    EigAnalysis.Spiral.Wn           = wnLat(spiralInd);
    EigAnalysis.Spiral.Wn           = dampLat(spiralInd);
    EigAnalysis.Spiral.Pole         = polesLat(spiralInd);
    EigAnalysis.Roll.Wn             = wnLat(rollInd);
    EigAnalysis.Roll.Damping        = dampLat(rollInd);
    EigAnalysis.Roll.Pole           = polesLat(rollInd);
    
    % Find indexes of eigenvectors for longitudinal modes mode
    [~, dtchVecInd] = ismembertol(real(EigAnalysis.DutchRoll.Pole), ...
        real(eigValsVecLat), 0.01);
    [~, sprlVecInd] = ismembertol(real(EigAnalysis.Spiral.Pole), ...
        real(eigValsVecLat), 0.01);
    [~, rollVecInd] = ismembertol(real(EigAnalysis.Roll.Pole), ...
        real(eigValsVecLat), 0.01);
    
    % Eigenvectors for longitudinal modes
    eigenVecDutchRoll   = Ulat(:,dtchVecInd(1));
    eigenVecSpiral      = Ulat(:,sprlVecInd(1));
    eigenVecRoll        = Ulat(:,rollVecInd(1));
    
    % Store longitudinal-directional eigenvectors
    EigAnalysis.DutchRoll.Eigenvector   = eigenVecDutchRoll;
    EigAnalysis.Spiral.Eigenvector      = eigenVecSpiral;
    EigAnalysis.Roll.Eigenvector        = eigenVecRoll;
    
    % Plot eigenvectors
    if plsPlotEigenVecs
        plotEigenvectors(EigAnalysis, rowsLon, rowsLat);
    end
 
    % Pring analysis to screen if desired
    if printAnalysis
        
        % Longitudinal
        fprintf('\n')
        fprintf('                Longitudinal-Directional Eigen Analysis');
        fprintf('\n\n');
        damp(Alon)
        
        % Lateral
        fprintf('\n')
        fprintf('-------------------------------------------------------');
        fprintf('----------------\n\n');
        fprintf('                  Lateral-Directional Eigen Analysis');
        fprintf('\n\n');
        damp(Alat(1:4,1:4))
    end
end