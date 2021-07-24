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

P = 2.294; % system with no gain corsses real at -0.436, so the gain needed to make unstable is -1/-0.436
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

