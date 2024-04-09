clear;
close all;
clc;

%% PARAMETERS
Parameters;

%% STATE SPACE MODELS
a11 = -lambda*(R_s*L_r + R_r*L_s)+1i*omega_r;
a12 = lambda*(R_r - 1i*L_r*omega_r);
a21 = -R_s;

A = [a11 a12; a21 0];
B = [lambda*L_r; 1];
C = [1 0];

sys = ss(A,B,C,[]);

%% OBSERVER
P = [-omega_r*2, -omega_r*1.5];

L = place(A',C',P)';

At = A - L*C;
Bt = [B, L];
Ct = [C; eye(2)];
sysObs = ss(At,Bt,Ct,[]);

%% EXTENDED OBSERVER
% b = -2000*12.5;
b = -50;
L_ext = -[2*b; b/(lambda*L_r)];

sysOE = ss((A - L_ext*C),[B,L_ext],[C;eye(2)],[]);

%% SIMULATION
t_sample = 1e-3;
t = (0:t_sample:1)';
u = zeros(length(t),1);
u_index = round(0.5/t_sample);

% Constant value --------
% u(u_index:end) = u_S + 1i*u_S;
% Sinosidial value ------
u(u_index:end) = u_S*sin(2*pi*f.*t(1:end-u_index+1)) + 1i*u_S*cos(2*pi*f.*t(1:end-u_index+1));

[y, ~,x] = lsim(sys,u,t);
[xhat, ~] = lsim(sysObs,[u,y],t);
[xhatE,tOut] = lsim(sysOE,[u,y],t);

%% SPLIT IMAGINERY AND REAL STATE SPACE MODEL
sysIm = ss(imag(A),imag(B),[0 1],[]);
sysRe = ss(real(A),real(B),[0 1],[]);

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
subplot(1,2,1)
hold on
plot(tOut, (x(:,1)),'Color','#0007D8','LineWidth',1.5,'DisplayName','i_S')
plot(tOut, (xhat(:,2)),'Color','#0C7616','LineWidth',1.5,'LineStyle','--','DisplayName','i_S (observer)')
plot(tOut, (xhatE(:,2)),'Color','#000000','LineWidth',1.5,'LineStyle','--','DisplayName','i_S (ext. observer)')
hold off
legend
xlabel('Time / sec')
ylabel('Stator current i_S / A')
title('Stator current (alpha)')

subplot(1,2,2)
hold on
plot(tOut, (x(:,2)),'Color','#0007D8','LineWidth',1.5,'DisplayName','Stator flux')
plot(tOut, (xhat(:,3)),'Color','#0C7616','LineWidth',1.5,'LineStyle','--','DisplayName','Stator flex (observer)')
plot(tOut, (xhatE(:,3)),'Color','#000000','LineWidth',1.5,'LineStyle','--','DisplayName','Stator flex (ext. observer)')
hold off
legend
xlabel('Time / sec')
ylabel('Stator flux / C')
title('Stator flux (alpha)')
