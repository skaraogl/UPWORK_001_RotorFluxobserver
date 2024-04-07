clear;
close all;
clc;

Poles;
Poles_V2;

figure
hold on
for i = 1:length(p1)
    plot(p1{i},'x','Color','#000000')
    plot(p2{i},'o','Color','#0EBD09')
end
legend('Paper: Speed Sensorless Control','Paper: Model predictive torque control')