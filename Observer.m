clear;
close all;
clc;
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
% Stator and rotor resistance / Ohm
R_s = 0.087;
R_r = 0.228;

lambda = 1/(L_s*L_r - L_m^2);

% Number of pole pairs
P = 2;
% Electrical stator frequency / Hz
f = (P*N_r)/120;

%% STATE SPACE MODELS
a11 = -lambda*(R_s*L_r + R_r*L_s)+1i*omega_r;
a12 = lambda*(R_r - 1i*L_r*omega_r);
a21 = -R_s;

A = [a11 a12; a21 0];
B = [lambda*L_r; 1];
C = [1 0];

sys = ss(A,B,C,[]);

%% OBSERVER
P = [omega_r*1, omega_r*0];
% P = [omega_r*1];

L = place(A',C',P)';

At = A - L*C;
Bt = [B, L];
Ct = [C; eye(2)];
sysObs = ss(At,Bt,Ct,[]);

%% SIMULATION
t_sample = 1e-3;
t = (0:t_sample:2)';
u = zeros(length(t),1);
u_index = round(0.5/t_sample);

% Constant value --------
% u(u_index:end) = u_S + 1i*u_S;
% Sinosidial value ------
u(u_index:end) = u_S*sin(2*pi*f.*t(1:end-u_index+1)) + 1i*u_S*cos(2*pi*f.*t(1:end-u_index+1));

[y, ~,x] = lsim(sys,u,t);
[xhat, tOut] = lsim(sysObs,[u,y],t);

%% PLOTTING
% ----------- FIGURE 1 -----------
fig1 = figure('Position',[100 100 1200 600]);

subplot(1,2,1)
hold on
plot(tOut,real(u),'DisplayName','u (alpha)')
plot(tOut,imag(u),'--','DisplayName','u (beta)')
hold on
legend
xlabel('Time / sec')
ylabel('Voltage u / V')

subplot(1,2,2)
hold on
plot(tOut,real(y),'DisplayName','i (alpha)')
plot(tOut,imag(y),'DisplayName','i (beta)')
hold on
legend
xlabel('Time / sec')
ylabel('Current i / A')

% ----------- FIGURE 2 -----------

fig2 = figure('Position',[100 100 1200 800]);
subplot(2,2,1)
hold on
plot(tOut, real(x(:,2)),'DisplayName','Stator flux (alpha)')
plot(tOut, real(xhat(:,3)),'--','DisplayName','Stator flex (estimated - alpha)')
hold off
legend
xlabel('Time / sec')
ylabel('Stator flux / C')
title('Stator flux (alpha)')

subplot(2,2,2)
plot(tOut, real(x(:,2)) - real(xhat(:,3)))
xlabel('Time / sec')
ylabel('Stator flux / C')
title('Estimation error of stator flux (alpha)')

subplot(2,2,3)
hold on
plot(tOut, imag(x(:,2)),'DisplayName','Stator flux (beta)')
plot(tOut, imag(xhat(:,3)),'--','DisplayName','Stator flex (estimated - beta)')
hold off
legend
xlabel('Time / sec')
ylabel('Stator flux / C')
title('Stator flux (alpha)')

subplot(2,2,4)
plot(tOut, imag(x(:,2)) - imag(xhat(:,3)))
xlabel('Time / sec')
ylabel('Stator flux / C')
title('Estimation error of stator flux (beta)')

figure
hold on
pzmap(sys)
pzmap(sysObs)
hold off