clear;
clc;
clf;
clf reset;
close all;

% Load data
cruise1 = load('cruise1');
cruise2 = load('cruise2');
approach1 = load('approach1');
approach2 = load('approach2');

% Unpack eigenvalues
phugEigVal_c1 = cruise1.EigAnalysis.Phugoid.Pole;
shrtEigVal_c1 = cruise1.EigAnalysis.ShortPeriod.Pole;
attiEigVal_c1 = cruise1.EigAnalysis.AttitudeConDi.Pole;
dtchEigVal_c1 = cruise1.EigAnalysis.DutchRoll.Pole;
sprlEigVal_c1 = cruise1.EigAnalysis.Spiral.Pole;
rollEigVal_c1 = cruise1.EigAnalysis.Roll.Pole;
phugEigVal_c2 = cruise2.EigAnalysis.Phugoid.Pole;
shrtEigVal_c2 = cruise2.EigAnalysis.ShortPeriod.Pole;
attiEigVal_c2 = cruise2.EigAnalysis.AttitudeConDi.Pole;
dtchEigVal_c2 = cruise2.EigAnalysis.DutchRoll.Pole;
sprlEigVal_c2 = cruise2.EigAnalysis.Spiral.Pole;
rollEigVal_c2 = cruise2.EigAnalysis.Roll.Pole;
phugEigVal_a1 = approach1.EigAnalysis.Phugoid.Pole;
shrtEigVal_a1 = approach1.EigAnalysis.ShortPeriod.Pole;
attiEigVal_a1 = approach1.EigAnalysis.AttitudeConDi.Pole;
dtchEigVal_a1 = approach1.EigAnalysis.DutchRoll.Pole;
sprlEigVal_a1 = approach1.EigAnalysis.Spiral.Pole;
rollEigVal_a1 = approach1.EigAnalysis.Roll.Pole;
phugEigVal_a2 = approach2.EigAnalysis.Phugoid.Pole;
shrtEigVal_a2 = approach2.EigAnalysis.ShortPeriod.Pole;
attiEigVal_a2 = approach2.EigAnalysis.AttitudeConDi.Pole;
dtchEigVal_a2 = approach2.EigAnalysis.DutchRoll.Pole;
sprlEigVal_a2 = approach2.EigAnalysis.Spiral.Pole;
rollEigVal_a2 = approach2.EigAnalysis.Roll.Pole;

figure;
hold on
p1 = plot(real(shrtEigVal_c1), imag(shrtEigVal_c1),'+m', 'MarkerSize', 10);
p2 = plot(real(shrtEigVal_c2), imag(shrtEigVal_c2),'+g', 'MarkerSize', 10);
p3 = plot(real(shrtEigVal_a1), imag(shrtEigVal_a1),'+r', 'MarkerSize', 10);
p4 = plot(real(shrtEigVal_a2), imag(shrtEigVal_a2),'+b', 'MarkerSize', 10);
xlabel('Re');
ylabel('Im');
set(gcf, 'Color', [1 1 1]);
set(gca, 'Color', [1 1 1]); 
xlim([-4 0])
grid on
gridxy(0,0);
hleg1 = legend([p1, p2, p3, p4], 'SPM (cg:1, 220kts)', ...
    'SPM (cg:2, 220kts)', 'SPM (cg:1, 90kts)', 'SPM (cg:2, 90kts)');
set(hleg1, 'Location', 'Best');
    
figure;
hold on
p5 = plot(real(phugEigVal_c1), imag(phugEigVal_c1),'om', 'MarkerSize', 10);
p6 = plot(real(attiEigVal_c1), imag(attiEigVal_c1),'*m', 'MarkerSize', 10);
p7 = plot(real(phugEigVal_c2), imag(phugEigVal_c2),'og', 'MarkerSize', 10);
p8 = plot(real(attiEigVal_c2), imag(attiEigVal_c2),'*g', 'MarkerSize', 10);
p9 = plot(real(phugEigVal_a1), imag(phugEigVal_a1),'or', 'MarkerSize', 10);
p10 = plot(real(attiEigVal_a1), imag(attiEigVal_a1),'*r', 'MarkerSize', 10);
p11 = plot(real(phugEigVal_a2), imag(phugEigVal_a2),'ob', 'MarkerSize', 10);
p12 = plot(real(attiEigVal_a2), imag(attiEigVal_a2),'*b', 'MarkerSize', 10);
xlabel('Re');
ylabel('Im');
set(gcf, 'Color', [1 1 1]);
set(gca, 'Color', [1 1 1]);
grid on
gridxy(0,0);
hleg2 = legend([p5, p6, p7, p8, p9, p10, p11, p12], ...
    'PM (cg:1, 220kts)', 'Attitude Con/Di (cg:1, 220kts)', ...
    'PM (cg:2, 220kts)', 'Attitude Cone/Di (cg:2, 220kts)', ...
    'PM (cg:1, 90kts)', 'Attitude Con/Di (cg:1, 90kts)', ...
    'PM (cg:2, 90kts)', 'Attitude Con/Di (cg:2, 90kts)');
set(hleg2, 'Location', 'Best');

figure;
hold on
p13 = plot(real(rollEigVal_c1), imag(rollEigVal_c1), 'pm', 'MarkerSize', 10);
p14 = plot(real(rollEigVal_c2), imag(rollEigVal_c2), 'pg', 'MarkerSize', 10);
p15 = plot(real(rollEigVal_a1), imag(rollEigVal_a1), 'pr', 'MarkerSize', 10);
p16 = plot(real(rollEigVal_a2), imag(rollEigVal_a2), 'pb', 'MarkerSize', 10);
xlabel('Re');
ylabel('Im');
set(gcf, 'Color', [1 1 1]);
set(gca, 'Color', [1 1 1]);
xlim([-1.75 0])
ylim([0 1])
grid on
gridxy(0,0);
hleg3 = legend([p13, p14, p15, p16], 'Roll (cg:1, 220kts)', ...
    'Roll (cg:2, 220kts)', 'Roll (cg:1, 90kts)', 'Roll (cg:2, 90kts)');
set(hleg3, 'Location', 'Best');

figure;
hold on
p17 = plot(real(dtchEigVal_c1), imag(dtchEigVal_c1), '^m', 'MarkerSize', 10);
p18 = plot(real(sprlEigVal_c1), imag(sprlEigVal_c1), 'xm', 'MarkerSize', 10);
p19 = plot(real(dtchEigVal_c2), imag(dtchEigVal_c2), '^g', 'MarkerSize', 10);
p20 = plot(real(sprlEigVal_c2), imag(sprlEigVal_c2), 'xg', 'MarkerSize', 10);
p21 = plot(real(dtchEigVal_a1), imag(dtchEigVal_a1), '^r', 'MarkerSize', 10);
p22 = plot(real(sprlEigVal_a1), imag(sprlEigVal_a1), 'xr', 'MarkerSize', 10);
p23 = plot(real(dtchEigVal_a2), imag(dtchEigVal_a2), '^b', 'MarkerSize', 10);
p24 = plot(real(sprlEigVal_a2), imag(sprlEigVal_a2), 'xb', 'MarkerSize', 10);
xlabel('Re');
ylabel('Im');
set(gcf, 'Color', [1 1 1]);
set(gca, 'Color', [1 1 1]);
xlim([-0.11 0])
grid on
gridxy(0,0);
hleg4 = legend([p17, p18, p19, p20, p21, p22, p23, p24], ...
    'Dutch Roll (cg:1, 220kts)', 'Spiral (cg:1, 220kts)', ...
    'Dutch Roll (cg:2, 220kts)', 'Spiral (cg:2, 220kts)', ...
    'Dutch Roll (cg:1, 90kts)', 'Spiral (cg:1, 90kts)', ...
    'Dutch Roll (cg:2, 90kts)', 'Spiral (cg:2, 90kts)');
set(hleg4, 'Location', 'Best');