function bankCondition(nPts, U, Alpha0, A0, WingProps, TailProps, Model)

    % Aircraft pitch angle of 2 degrees (rad)
    alpha_pitch = deg2rad(2);

    % Call function for banked turn lifting line theory on wing
    [CwB, WingAnglesB, ~, BankWing] = liftingLineBankWing(nPts, ...
                                      alpha_pitch, Alpha0, A0, WingProps, U);

    % Unpack wing bank properties struct
    gamma_w     = BankWing.Gamma;
    effAlpha_w  = BankWing.EffectiveAlpha;
    y_wing_bank = BankWing.SpanPos;

    % Unpack wing coefficient struct
    CL_w_bank   = CwB.CL;
    Cdi_w_bank  = CwB.Cdi;

    % Stabilator deflection angle during sustained bank (rad)
    deflec_up_bank = deg2rad(3);

    % Angle of attack of tail during sustained bank (rad)
    alpha_t_bank = alpha_pitch - deflec_up_bank;

    [CtB, TailAnglesB, ~, BankTail] = liftingLineTailBank(nPts, ...
                                      alpha_t_bank, U, Alpha0, A0, ...
                                      WingProps, TailProps, WingAnglesB);   

    % Unpack wing bank properties struct
    gamma_t     = BankTail.Gamma;
    effAlpha_t  = BankTail.EffectiveAlpha;
    y_tail_bank = BankTail.SpanPos;


    % Unpack tailplane coefficient struct
    CL_t_bank   = CtB.CL;
    Cdi_t_bank  = CtB.Cdi;

    % Re-normalise w.r.t wing reference area
    CL_t_norm_bank  = (CL_t_bank.*(TailProps.TailArea.*mean(Model.DynP)))./...
                      (WingProps.WingArea.*mean(Model.DynP));
    Cdi_t_norm_bank = (Cdi_t_bank.*(TailProps.TailArea.*mean(Model.DynP)))./...
                      (WingProps.WingArea.*mean(Model.DynP));

    % Total lift during sustained bank
    CL_total_bank = CL_w_bank + CL_t_norm_bank;

    % Total induced drag during sustained bank
    Cdi_total_bank = Cdi_w_bank + Cdi_t_norm_bank;

    % Plot downwash angles - BANKED TURN
    figure;
    plot(y_wing_bank,rad2deg(WingAnglesB.Downwash),'-x');
    xlabel('Spanwise Position (m)')
    ylabel('Downwash Angle (deg)')
    set(gca, 'XLimSpec', 'Tight');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    grid on

    % Lift distribution over wing with effective angle of attack
    figure;
    yyaxis left
    plot(y_wing_bank,gamma_w)
    ylabel('Circulation')
    xlabel('Spanwise Position (m)');
    yyaxis right
    plot(y_wing_bank,rad2deg(effAlpha_w));
    ylabel('Local Angle of Attack (deg)');
    grid on
    set(gca, 'XLimSpec', 'Tight');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);

    % Lift distribution over tail with effective angle of attack
    figure;
    yyaxis left
    plot(y_tail_bank,gamma_t);
    ylabel('Circulation')
    xlabel('Spanwise Position (m)');
    yyaxis right
    plot(y_tail_bank,rad2deg(effAlpha_t));
    ylabel('Local Angle of Attack (deg)');
    grid on
    set(gca, 'XLimSpec', 'Tight');
    set(gcf, 'Color', [1 1 1]);
    set(gca, 'Color', [1 1 1]);
    
end