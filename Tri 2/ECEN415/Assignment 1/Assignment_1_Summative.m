%% Question 1 %%
clc;
clear;

t_d=-0.5;
s = tf('s');

D = exp(s*t_d);

figure(1);
bode(D);

%% 
clc;
clear;

s = tf('s');

figure(1)

D = exp(s * -0.00027);
D_pade = pade(D, 1);

bode(D_pade);
legend("Delay = 270\mu s");

%%
clc;
clear;

s = tf('s');

figure(1)

D = exp(s * -0.00026);
D_pade = pade(D, 2);

bode(D_pade);
legend("Delay = 261\mu s");