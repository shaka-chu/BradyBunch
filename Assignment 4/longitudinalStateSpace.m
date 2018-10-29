function [Alon, Blon, Params] = longitudinalStateSpace(flightCond) 
    
    % Load based on chosen flight condition
    switch flightCond

        % 90 kts, nominal CG position
        case 'approach1'

            % Load .mat file for CG2 at 220 kts
            load A_lon_90Kn_500ft_CG1.mat

            % Load aircraft properties
            Params = aero3560_LoadFlightDataPC9_nominalCG1;

        % 90 kts, secondary CG position   
        case 'approach2'

            % Load .mat file for CG2 at 220 kts
            load A_lon_90Kn_500ft_CG2.mat

            % Load aircraft properties
            Params = aero3560_LoadFlightDataPC9_CG2;

        % 220 kts, nominal CG position   
        case 'cruise1'

            % Load .mat file for CG2 at 220 kts
            load A_lon_220Kn_500ft_CG1.mat

            % Load aircraft properties
            Params = aero3560_LoadFlightDataPC9_nominalCG1;

        % 220 kts, secondary CG position
        case 'cruise2'

            % Load .mat file for CG2 at 220 kts
            load A_lon_220Kn_500ft_CG2.mat

            % Load aircraft properties
            Params = aero3560_LoadFlightDataPC9_CG2; 
    end

    % Rename state space matrices
    Alon = A_lon5;
    Blon = B_lon5;
end