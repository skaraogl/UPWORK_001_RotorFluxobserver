% clear;
% close all;
% clc;

Parameters;

sigma = 1 - L_m^2/(L_s*L_r);
T_r = L_r/R_r;

% Rated Motor Frequency / Hz
f_rtd = 50;
% Rated Motor Speed / rpm
N_rtd = 120*f_rtd/P;
% Rated Motor Angular Velocity / rad/sec
omega_rtd = 2*pi*(N_rtd/60);

% OMEGA = linspace(0,omega_rtd,20)';
OMEGA = linspace(-omega_rtd,omega_rtd,40)';

%% STATE SPACE MODELS
% figure
% hold on

for i = 1:length(OMEGA)
a11 = -(R_s + (L_m/L_r)^2*R_r)/(sigma*L_s);
a12 = L_m/(sigma*L_s*L_r)*(1/T_r - 1i*OMEGA(i));
a21 = L_m/T_r;
a22 = -(1/T_r - 1i*OMEGA(i));

A = [a11 a12; a21 a22];
B = [1/(sigma*L_s); 0];
C = [0 1];

sys = ss(A,B,C,[]);
% sysRe = ss(real(A),real(B),C,[]);
% sysIm = ss(imag(A),imag(B),C,[]);

[p2{i,1},~] = pzmap(sys);
% pzmap(sysRe)
% pzmap(sysIm)
end
% hold off
% legend