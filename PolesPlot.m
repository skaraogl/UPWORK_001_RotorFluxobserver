clear;
close all;
clc;

Poles;
% Poles_V2;

figure
hold on
for i = 1:length(pSys)
    plot(pSys{i},'x','Color','#000000')
    plot(pObs{i},'x','Color','#D51313')
    plot(pEO{i},'x','Color','#D56813')
end
xlabel('Re')
ylabel('Im')
xlim([-inf 0])
legend('System','Observer','Extended Observer')