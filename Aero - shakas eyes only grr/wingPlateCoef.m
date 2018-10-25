function Cf_avg = wingPlateCoef(Re, TransPoints)

    % Unpack transition points struct
    xtrc_upper_root  = TransPoints.Wing.RootUpper;
    xtrc_lower_root  = TransPoints.Wing.RootLower;
    xtrc_upper_tip   = TransPoints.Wing.TipUpper;
    xtrc_lower_tip   = TransPoints.Wing.TipLower;

    % Unpack Reynolds number struct
    Re_root = Re.WingRoot;
    Re_tip  = Re.WingTip;
    
%     % Obtain x_tr/c for wing upper and lower surfaces at root and tip
%     xtrc_upper_root  = PolarRoot.Top_xtr;
%     xtrc_lower_root  = PolarRoot.Bot_xtr;
%     xtrc_upper_tip   = PolarTip.Top_xtr;
%     xtrc_lower_tip   = PolarTip.Bot_xtr;
    
    % Obtain x0/c for wing upper and lower surfaces at root and tip
    x0c_upper_root   = 36.9.*(xtrc_upper_root.^(0.625)).* ...
                       (1/Re_root)^(0.275);
    x0c_lower_root   = 36.9.*(xtrc_lower_root.^(0.625)).* ...
                       (1/Re_root)^(0.275);
    x0c_upper_tip    = 36.9.*(xtrc_upper_tip.^(0.625)).* ...
                       (1/Re_tip)^(0.275);
    x0c_lower_tip    = 36.9.*(xtrc_lower_tip.^(0.625)).* ...
                       (1/Re_tip)^(0.275);
                      
    % Calculate plate drag coefficients for wing upper and lower surfaces
    % at root and tip
    Cf_upper_root = (0.074/Re_root^(0.2)).*(1 - ...
                    (xtrc_upper_root - x0c_upper_root)).^(0.8);
    Cf_lower_root = (0.074/Re_root^(0.2)).*(1 - ...
                    (xtrc_lower_root - x0c_lower_root)).^(0.8);
    Cf_upper_tip  = (0.074/Re_tip^(0.2)).*(1 - ...
                    (xtrc_upper_tip - x0c_upper_tip)).^(0.8);
    Cf_lower_tip  = (0.074/Re_tip^(0.2)).*(1 - ...
                    (xtrc_lower_tip - x0c_lower_tip)).^(0.8);
                  
    % Average plate coefficient for root and tip wing airfoils
    Cf_avg_root   = 1/2*(Cf_upper_root + Cf_lower_root);
    Cf_avg_tip    = 1/2*(Cf_upper_tip + Cf_lower_tip);
    
    % Avergage plate coefficient for wing
    Cf_avg = 1/2*(Cf_avg_root + Cf_avg_tip); 

end