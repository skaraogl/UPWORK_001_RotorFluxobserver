%% PARAMETERS
% Electrical rotor rpm / rpm
N_r = 1500;
% Electrical rotor speed / rad/sec
omega_r = 2*pi*(N_r/60);
% Applied voltage / V
u_S = 780;
% Magnetizing/mutual, sator and rotor inductance / H
L_m = 34.7e-3;
L_s = 8e-4 + L_m;
L_r = 8e-4 + L_m;
% L_m = 0.2687;
% L_s = 0.2815;
% L_r = 0.2815;
% Stator and rotor resistance / Ohm
R_s = 0.087;
R_r = 0.228;
% R_s = 2.845;
% R_r = 2.413;

lambda = 1/(L_s*L_r - L_m^2);

% Number of pole pairs
P = 2;
% Electrical stator frequency / Hz
f = (P*N_r)/120;



