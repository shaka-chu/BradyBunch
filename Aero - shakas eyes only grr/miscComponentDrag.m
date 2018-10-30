function Dq_misc = miscComponentDrag

    % Drag coefficient of landing gear with fender (w.r.t. wheel area)
    % Hoerner, 1965 (p 13-14, figure 26)
    CD_gear = 0.2;

    % Nose and main wheels (3)
    wheelWidth = convlength(6, 'in', 'm');
    wheelHeight = convlength(15, 'in', 'm');
    S_wheel = wheelWidth*wheelHeight;
    Dq_gear = CD_gear*S_wheel;
    Dq_gearTotal = 3*Dq_gear;

    % Drag coefficient of antenna (w.r.t. frontal area of antenna)
    % Hoerner, 1965 (p 15-21, figure 27)
    CD_antenna = 0.1;

    % VTP-17 antenna (2)
    txAntennaHeight = convlength(8.5, 'in', 'm');
    txAntennaWidth = convlength(0.218, 'in', 'm');
    S_txAntenna = txAntennaHeight*txAntennaWidth;
    Dq_txAntenna = CD_antenna*S_txAntenna;

    % Blade antenna (1)
    txBladeHeight = convlength(13, 'in', 'm');
    txBladeWidth = convlength(0.75, 'in', 'm');
    S_txBlade= txBladeHeight*txBladeWidth;
    Dq_txBlade = CD_antenna*S_txBlade;

    % VRP-37 antenna (2)
    rxAntennaHeight = convlength(0.218, 'in', 'm');
    rxAntennaWidth = convlength(14.465, 'in', 'm');
    S_rxAntenna = rxAntennaHeight*rxAntennaWidth;
    Dq_rxAntenna = CD_antenna*S_rxAntenna;

    % Total antenna drag
    Dq_antennaTotal = 2*Dq_txAntenna + Dq_txBlade + 2*Dq_rxAntenna;

    % Drag coefficient of the propeller, (w.r.t. developed blade area)
    % Sensenich 74DM6-0-58 Propeller
    propellerBladeAngle = ...
        mean(deg2rad([33.55 28.3 23.1 19.04 16.1 14.6]));
    propellerDia = convlength(74, 'in', 'm');
    propellerMaxChord = convlength(5.5, 'in', 'm');
    S_propeller = propellerDia*propellerMaxChord/2;

    % Hoerner, 1965 (p 13-21, equation 20)
    CD_propeller = 0.1 + cos(propellerBladeAngle)^2;
    Dq_propeller = CD_propeller*S_propeller;

    % Drag coefficient of the tail beacon, approximate as a cylinder
    % (w.r.t. frontal area)
    % Hoerner, 1965 (p 13-19, figure 50)
    CD_beacon = 1.2;
    beaconHeight = convlength(3.74, 'in', 'm');
    beaconWidth = convlength(2.6, 'in', 'm');
    S_beacon = beaconHeight*beaconWidth;
    Dq_beacon = CD_beacon*S_beacon;

    % Drag coefficient of the side step
    CD_sidestep = 0.2;
    sidestepWidth = 0.12;
    sidestepHeight = 0.16;
    sidestepThickness = 0.02;
    S_sidestep = sidestepWidth*sidestepThickness ...
        + sidestepHeight*sidestepThickness;
    Dq_sidestep = CD_sidestep*S_sidestep;

    % Calculate total "drag area" of the aircraft
    Dq_misc = Dq_gearTotal + Dq_antennaTotal + Dq_propeller ...
        + Dq_beacon + Dq_sidestep;
end