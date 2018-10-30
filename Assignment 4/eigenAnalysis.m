function EigAnalysis = eigenAnalysis(Alon, Alat, printAnalysis)

    % Obtain eigenvectors and diagonal matrix of eigenvalues for
    % longitudinal state space
    eigenvalsLon = eig(Alon);
    
    % Obtain frequency, damping and poles of longitudinal state space
    [wnLon, dampLon, polesLon] = damp(Alon);
    
    % Obtain frequency, damping and poles of longitudinal state space
    [wnLat, dampLat, polesLat] = damp(Alat([1:4],[1:4]));
    
    % Longitudinal-directional data in order of frequency
    [maxFreqLon, maxIndLon] = maxk(wnLon, length(eigenvalsLon));
    
    % Store longitudinal-directional eigen analysis in struct
    EigAnalysis.Phugoid.Wn              = maxFreqLon(1);
    EigAnalysis.Phugoid.Damping         = dampLon(maxIndLon(1));
    EigAnalysis.Phugoid.Pole            = polesLon(maxIndLon(1));
    EigAnalysis.ShortPeriod.Wn          = maxFreqLon(3);
    EigAnalysis.ShortPeriod.Wn          = dampLon(maxIndLon(3));
    EigAnalysis.ShortPeriod.Pole        = polesLon(maxIndLon(3));
    EigAnalysis.AttituedConDi.Wn        = maxFreqLon(5);
    EigAnalysis.AttituedConDi.Damping   = dampLon(maxIndLon(5));
    EigAnalysis.AttituedConDi.Pole      = polesLon(maxIndLon(5));
    
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
        damp(Alat([1:4],[1:4]))
    end
end