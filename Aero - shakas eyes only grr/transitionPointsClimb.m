function TransPoints = transitionPointsClimb

    % Load wing root data
    fileName            = 'climbWingRoot_text.txt';
    fid                 = fopen(fileName,'r');
    wing_root_xfoil     = textscan(fid,'%f %f %f %f %f %f %f', ...
                          'Headerlines',2);
    top_xtr_wing_root   = wing_root_xfoil{6};
    bot_xtr_wing_root   = wing_root_xfoil{7};
    fclose('all');

    % Load wing tip data
    fileName            = 'climbWingTip_text.txt';
    fid                 = fopen(fileName,'r');
    wing_tip_xfoil      = textscan(fid,'%f %f %f %f %f %f %f', ...
                         'Headerlines',2);
    top_xtr_wing_tip    = wing_tip_xfoil{6};
    bot_xtr_wing_tip    = wing_tip_xfoil{7};
    fclose('all');

    % Load tail root data
    fileName            = 'climbTailRoot_text.txt';
    fid                 = fopen(fileName,'r');
    tail_root_xfoil     = textscan(fid,'%f %f %f %f %f %f %f', ...
                          'Headerlines',2);
    top_xtr_tail_root   = tail_root_xfoil{6};
    bot_xtr_tail_root   = tail_root_xfoil{7};
    fclose('all');
    
    % Load tail tip data
    fileName            = 'climbTailRoot_text.txt';
    fid                 = fopen(fileName,'r');
    tail_root_xfoil     = textscan(fid,'%f %f %f %f %f %f %f', ...
                          'Headerlines',2);
    top_xtr_tail_tip    = tail_root_xfoil{6};
    bot_xtr_tail_tip    = tail_root_xfoil{7};
    fclose('all');


    % Store transition point data in structs
    TransPoints.Wing.RootUpper  = top_xtr_wing_root;
    TransPoints.Wing.RootLower  = bot_xtr_wing_root;
    TransPoints.Wing.TipUpper   = top_xtr_wing_tip;
    TransPoints.Wing.TipLower   = bot_xtr_wing_tip;
    TransPoints.Tail.RootUpper  = top_xtr_tail_root;
    TransPoints.Tail.RootLower  = bot_xtr_tail_root;
    TransPoints.Tail.TipUpper   = top_xtr_tail_tip;
    TransPoints.Tail.TipLower   = bot_xtr_tail_tip;
    
end