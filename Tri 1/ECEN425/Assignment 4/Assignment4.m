clc
clear

load("q5-var.mat");         % Load the diamater, force, and strain values %
Area = 1/4 * pi*d^2;        % Calculate the area % 

Stress = Force/Area;        % Calculate stress %

plot(Strain, Stress)        % Plot the stress vs strain %
title("Stress vs Strain");
xlabel("Strain");
ylabel("Stress");
