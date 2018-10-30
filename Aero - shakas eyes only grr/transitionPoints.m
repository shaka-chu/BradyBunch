function TransPoints = transitionPoints

    % Load wing root data
    fileName            = 'wingRoot1_text.txt';
    fid                 = fopen(fileName,'r');
    wing_root_xfoil     = textscan(fid,'%f %f %f %f %f %f %f', ...
                          'Headerlines',2);
    top_xtr_wing_root   = wing_root_xfoil{6};
    bot_xtr_wing_root   = wing_root_xfoil{7};
    fclose('all');

    % Load wing tip data
    fileName            = 'wingTip1_text.txt';
    fid                 = fopen(fileName,'r');
    wing_tip_xfoil_1    = textscan(fid,'%f %f %f %f %f %f %f', ...
                         'Headerlines',2);
    top_xtr_wing_tip_1  = wing_tip_xfoil_1{6};
    bot_xtr_wing_tip_1  = wing_tip_xfoil_1{7};
    fclose('all');
    fileName            = 'wingTip2_text.txt';
    fid                 = fopen(fileName,'r');
    wing_tip_xfoil_2    = textscan(fid,'%f %f %f %f %f %f %f', ...
                         'Headerlines',2);
    top_xtr_wing_tip_2  = wing_tip_xfoil_2{6};
    bot_xtr_wing_tip_2  = wing_tip_xfoil_2{7};
    fclose('all');

    % Concatenate wing tip data
    top_xtr_wing_tip    = [top_xtr_wing_tip_1; top_xtr_wing_tip_2];
    bot_xtr_wing_tip    = [bot_xtr_wing_tip_1; bot_xtr_wing_tip_2];

    % Load tail root data
    fileName            = 'tail1_text.txt';
    fid                 = fopen(fileName,'r');
    tail_root_xfoil_1   = textscan(fid,'%f %f %f %f %f %f %f', ...
                          'Headerlines',2);
    top_xtr_tail_1      = tail_root_xfoil_1{6};
    bot_xtr_tail_1      = tail_root_xfoil_1{7};
    fclose('all');
    fileName            = 'tail2_text.txt';
    fid                 = fopen(fileName,'r');
    tail_root_xfoil_2   = textscan(fid,'%f %f %f %f %f %f %f', ...
                          'Headerlines',2);
    top_xtr_tail_2      = tail_root_xfoil_2{6};
    bot_xtr_tail_2      = tail_root_xfoil_2{7};
    fclose('all');
    
    % Concatenate tailplane data
    top_xtr_tail        = [top_xtr_tail_1; top_xtr_tail_2];
    bot_xtr_tail        = [bot_xtr_tail_1; bot_xtr_tail_2];

    % Store transition point data in structs
    TransPoints.Wing.RootUpper  = top_xtr_wing_root;
    TransPoints.Wing.RootLower  = bot_xtr_wing_root;
    TransPoints.Wing.TipUpper   = top_xtr_wing_tip;
    TransPoints.Wing.TipLower   = bot_xtr_wing_tip;
    TransPoints.Tail.Upper      = top_xtr_tail;
    TransPoints.Tail.Lower      = bot_xtr_tail;
    
end