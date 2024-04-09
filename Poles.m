Parameters;

% Rated Motor Frequency / Hz
f_rtd = 50;
% Rated Motor Speed / rpm
N_rtd = 120*f_rtd/P;
% Rated Motor Angular Velocity / rad/sec
omega_rtd = 2*pi*(N_rtd/60);

% OMEGA = linspace(0,omega_rtd,20)';
OMEGA = linspace(-omega_rtd,omega_rtd,40)';

% Parameter for Extended Observer
% b = -2000*12.5;
b = -50;

%% STATE SPACE MODELS

for i = 1:length(OMEGA)
% System model
a11 = -lambda*(R_s*L_r + R_r*L_s)+1i*OMEGA(i);
a12 = lambda*(R_r - 1i*L_r*OMEGA(i));
a21 = -R_s;

A = [a11 a12; a21 0];
B = [lambda*L_r; 1];
C = [1 0];

sys = ss(A,B,C,[]);

% Observer
poles = [-OMEGA(i)*2, -OMEGA(i)*1.5];
% poles = [-OMEGA(i)*2+1i*OMEGA(i), -OMEGA(i)*2-1i*OMEGA(i)];
L = place(A',C',poles)';
At = A - L*C;
Bt = [B, L];
Ct = [C; eye(2)];
sysObs = ss(At,Bt,Ct,[]);

% Extended observer
L_ext = -[2*b; b/(lambda*L_r)];
sysEO = ss((A - L_ext*C),[B,L_ext],[C;eye(2)],[]);

% Calculate Poles
[pSys{i,1},~] = pzmap(sys);
[pObs{i,1},~] = pzmap(sysObs);
[pEO{i,1},~] = pzmap(sysEO);

end