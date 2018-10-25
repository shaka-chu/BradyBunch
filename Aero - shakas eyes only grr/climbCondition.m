function climbCondition(n, nPts, U, Alpha0, A0, WingProps, TailProps, ...
    Model)

    % Aircraft climb angle (rad)
    alpha_climb = deg2rad(8);

    % Lifting line theory applied to main wing during climb
    [CwC, WingAnglesC, ~, ClimbWing] = liftingLineWing(n, nPts, ...
                                       alpha_climb, Alpha0, A0, WingProps, U);
                                   
    % Unpack wing climb properties struct
    gamma_w     = ClimbWing.Gamma;
    effAlpha_w  = ClimbWing.EffectiveAlpha;
    y_wing      = ClimbWing.SpanPos;

    % Stabilator deflection angle during climb (rad)
    deflec_up_climb = deg2rad(10);

    % Angle of attack of tail during climg (rad)
    alpha_t_climb = alpha_climb - deflec_up_climb;

    % Lifting line theory applied to tailplane
    [CtC, TailAnglesC, TailPropsC, ClimbTail] = liftingLineTail(n, nPts, U, ...
                                                alpha_t_climb, Alpha0, A0, ...
                                                WingProps, TailProps, ...
                                                WingAnglesC);
                                            
    % Unpack tail climb properties struct
    gamma_t     = ClimbTail.Gamma;
    effAlpha_t  = ClimbTail.EffectiveAlpha;
    y_tail      = ClimbTail.SpanPos;
                                            
    % Lift distribution over wing with effective angle of attack
    figure;
    yyaxis left
    plot(y_wing,gamma_w)
    ylabel('Circulation')
    xlabel('Spanwise Position (m)');
    yyaxis right
    plot(y_wing,rad2deg(effAlpha_w));
    ylabel('Local Angle of Attack (deg)');
    grid on
    set(gca, 'XLimSpec', 'Tight');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);

    % Lift distribution over tail with effective angle of attack
    figure;
    yyaxis left
    plot(y_tail,gamma_t);
    ylabel('Circulation')
    xlabel('Spanwise Position (m)');
    yyaxis right
    plot(y_tail,rad2deg(effAlpha_t));
    ylabel('Local Angle of Attack (deg)');
    grid on
    set(gca, 'XLimSpec', 'Tight');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
end