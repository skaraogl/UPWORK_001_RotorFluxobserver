% clear;
% close all;
% clc;

Parameters;

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
a11 = -lambda*(R_s*L_r + R_r*L_s)+1i*OMEGA(i);
a12 = lambda*(R_r - 1i*L_r*OMEGA(i));
a21 = -R_s;

A = [a11 a12; a21 0];
B = [lambda*L_r; 1];
C = [1 0];

sys = ss(A,B,C,[]);
% sysRe = ss(real(A),real(B),C,[]);
% sysIm = ss(imag(A),imag(B),C,[]);

[p1{i,1},~] = pzmap(sys);
% pzmap(sysRe)
% pzmap(sysIm)
end
% hold off
% legend