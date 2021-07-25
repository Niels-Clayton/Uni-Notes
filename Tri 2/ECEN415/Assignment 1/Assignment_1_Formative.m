%% Question 1 %%

G = 1;
s = tf('s');

G_open   = G*(20*(s^2+s+0.5))/(s*(s+1)*(s+10))
G_closed = feedback(G_open,1);

figure(1);
nyquist(G_open);
% pzmap(G_closed);

%%

G = 1;
s = tf('s');

G_open = G*(20*(s^2+s+0.5))/(s*(s-1)*(s+10))
G_closed = feedback(G_open,1);

figure(2);
nyquist(G_open);
% pzmap(G_closed);

%%

G = 1;
s = tf('s');

G_open = G*(s^2+3)/((s+1)^2)
G_closed = feedback(G_open,1);

figure(3);
nyquist(G_open);
% pzmap(G_closed);

%%

G = 1;
s = tf('s');

G_open = G*(3*(s+1))/(s*(s-10));
G_closed = feedback(G_open,1);

figure(4);
nyquist(G_open);
% pzmap(G_closed);

%% Question 2 %%
clc
clear

s = tf('s');

G_open = (4/(s+2));
G_open.IODelay = 0.2;

figure(1);
nyquist(G_open);

%%

P = 2.294; % system with no gain crosses real at -0.436, so the gain needed to make unstable is -1/-0.436
G_closed_stable = feedback(G_open, 1);
G_closed_unstable = feedback(G_open, P);


figure(2);
step(G_closed_stable);
title("Closed loop step fucntion stable");
legend("P=1");

figure(3);
step(G_closed_unstable);
title("Closed loop step fucntion unstable");
legend("P = 2.294");

%%

s = tf('s');

G_open = (4/(s+2));
G_open.IODelay = 0.61;

G_closed_unstable = feedback(G_open, 1);

figure(1);
step(G_closed_unstable);
title("Closed loop step fucntion unstable");
legend("Delay = 0.61");

figure(2);
nyquist(G_open);
legend("Delay = 0.61");

%% Question 3 %%
clc; 
clear;

s = tf('s');
t_step = 0.0001;

G = 1000/(s+1000);

opt = c2dOptions('Method','tustin');
G_bilinear = c2d(G, t_step, opt');

opt = c2dOptions('Method','tustin','PrewarpFrequency', 1000);
G_bilinear_warp = c2d(G, t_step, opt);

opt = c2dOptions('Method','impulse');
G_impulse = c2d(G, t_step, opt');

opt = c2dOptions('Method','matched');
G_matched = c2d(G, t_step, opt');

% Plot the step responses
figure(1);
hold on;
step(G);
step(G_bilinear)
step(G_bilinear_warp);
step(G_impulse);
step(G_matched);
legend("Continuious Time", "Bilinear", "Bilinear Prewarp", "Impulse Invariance", "Matched Pole Zero")
hold off;

% Plot the impulse responses
figure(2)
hold on
impulse(G);
impulse(G_bilinear)
impulse(G_bilinear_warp);
impulse(G_impulse);
impulse(G_matched);
legend("Continuious Time", "Bilinear", "Bilinear Prewarp", "Impulse Invariance", "Matched Pole Zero")
hold off

% Plot the bode diagram
figure(3)
hold on
bode(G);
bode(G_bilinear)
bode(G_bilinear_warp);
bode(G_impulse);
bode(G_matched);
legend("Continuious Time", "Bilinear", "Bilinear Prewarp", "Impulse Invariance", "Matched Pole Zero")
hold off
