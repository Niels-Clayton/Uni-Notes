%% Question 1 %%
clc;
clear;

t_d=-1;
s = tf('s');

D = exp(s*t_d);

figure(1);
bode(D);