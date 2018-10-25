function TransPoints = transitionPointsOld

    % Load wing root data
    fileName            = 'naca652415_w_root_1_text.txt';
    fid                 = fopen(fileName,'r');
    wing_root_xfoil_1   = textscan(fid,'%f %f %f %f %f %f %f', ...
                          'Headerlines',2);
    top_xtr_wing_root_1 = wing_root_xfoil_1{6};
    bot_xtr_wing_root_1 = wing_root_xfoil_1{7};
    fclose('all');
    fileName            = 'naca652415_w_root_2_text.txt';
    fid                 = fopen(fileName,'r');
    wing_root_xfoil_2   = textscan(fid,'%f %f %f %f %f %f %f', ...
                          'Headerlines',2);
    top_xtr_wing_root_2 = wing_root_xfoil_2{6};
    bot_xtr_wing_root_2 = wing_root_xfoil_2{7};
    fclose('all');
    fileName            = 'naca652415_w_root_3_text.txt';
    fid                 = fopen(fileName,'r');
    wing_root_xfoil_3   = textscan(fid,'%f %f %f %f %f %f %f', ...
                          'Headerlines',2);
    top_xtr_wing_root_3 = wing_root_xfoil_3{6};
    bot_xtr_wing_root_3 = wing_root_xfoil_3{7};
    fclose('all');

    % Concatenate wing root data
    top_xtr_wing_root = [top_xtr_wing_root_1; top_xtr_wing_root_2; ...
                         top_xtr_wing_root_3];
    bot_xtr_wing_root = [bot_xtr_wing_root_1; bot_xtr_wing_root_2; ...
                         bot_xtr_wing_root_3];

    % Load wing tip data
    fileName            = 'naca652415_w_tip_1_text.txt';
    fid                 = fopen(fileName,'r');
    wing_tip_xfoil_1   = textscan(fid,'%f %f %f %f %f %f %f', ...
                         'Headerlines',2);
    top_xtr_wing_tip_1 = wing_tip_xfoil_1{6};
    bot_xtr_wing_tip_1 = wing_tip_xfoil_1{7};
    fclose('all');
    fileName            = 'naca762415_w_tip_2_text.txt';
    fid                 = fopen(fileName,'r');
    wing_tip_xfoil_2   = textscan(fid,'%f %f %f %f %f %f %f', ...
                         'Headerlines',2);
    top_xtr_wing_tip_2 = wing_tip_xfoil_2{6};
    bot_xtr_wing_tip_2 = wing_tip_xfoil_2{7};
    fclose('all');

    % Concatenate wing tip data
    top_xtr_wing_tip = [top_xtr_wing_tip_1; top_xtr_wing_tip_2];
    bot_xtr_wing_tip = [bot_xtr_wing_tip_1; bot_xtr_wing_tip_2];

    % Load tail root data
    fileName            = 'naca652415_t_root_1_text.txt';
    fid                 = fopen(fileName,'r');
    tail_root_xfoil     = textscan(fid,'%f %f %f %f %f %f %f', ...
                          'Headerlines',2);
    top_xtr_tail_root   = tail_root_xfoil{6};
    bot_xtr_tail_root   = tail_root_xfoil{7};
    fclose('all');

    % Store transition point data in structs
    TransPoints.Wing.RootUpper  = top_xtr_wing_root;
    TransPoints.Wing.RootLower  = bot_xtr_wing_root;
    TransPoints.Wing.TipUpper   = top_xtr_wing_tip;
    TransPoints.Wing.TipLower   = bot_xtr_wing_tip;
    TransPoints.Tail.Upper      = top_xtr_tail_root;
    TransPoints.Tail.Lower      = bot_xtr_tail_root;
    
end