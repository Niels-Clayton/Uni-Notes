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

%% Question 2 %%
clc;
clear;

s = tf('s');
t_s = -1/1000;

D = pade(exp(s * t_s), 1);
sampler = (1 - D)/s;
G = 15 / ((s+1)*(s+2));

figure(1);
rlocus(G);
title('Root locus of unsampled system');

figure(2);
rlocus(sampler*G);
title('Root locus of sampled system');
legend("t_s = 1ms");

%%
clc;
clear;

s = tf('s');
t_s = -1/5.65

D = pade(exp(s * t_s), 1);
sampler = (1 - D)/s;
G = 15 / ((s+1)*(s+2));

figure(1);
margin(G);

figure(3)
margin(sampler*G);
legend("t_s = 177ms");

%%
clc;
clear;

% The phase margin is 10 degrees and a frequency of 10kHz
% Aliased signals must be atenuated by 40dB (corner frequency 1 decade before Niquist frequency)
% Must sample at least 10 times faster than unity gain frequency

s = tf('s');

% Using the exact 
f = 1807100;
phase = -inf;
while phase < -10
    t_s = -1/f;                      % sampling period 
    
    sampler = (1 - exp(s * t_s))/s; 
    wc = (f/20) * 2 * pi;            % Corner frequency 1 decade before Niquist frequency

    G_b = (wc^2)/(s^2 + (sqrt(2)*wc*s) + wc^2);
    
    [mag, phase] = bode(sampler*G_b, 10000*2*pi);
    
    f = f+1;                        % Increment sampling frequency
end
f

figure(1);
bode(sampler*G_b);
title('Sampler using pure delay');


% Using the pade approx
f = 1807100;
phase = -inf;
while phase < -10
    t_s = -1/f;                      % sampling period 
    
    D = pade(exp(s * t_s), 1);
    sampler = (1 - D)/s; 
    wc = (f/20) * 2 * pi;            % Corner frequency 1 decade before Niquist frequency

    G_b = (wc^2)/(s^2 + (sqrt(2)*wc*s) + wc^2);
    
    [mag, phase] = bode(sampler*G_b, 10000*2*pi);
    
    f = f+1;                        % Increment sampling frequency
end
f

figure(2);
bode(sampler*G_b);
title('Sampler using Pade approximated delay');

