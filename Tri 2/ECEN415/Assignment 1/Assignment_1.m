%%

G = 1;
s = tf('s');

G1 = (20*(s^2+s+0.5))/(s*(s+1)*(s+10))
figure(1);
nyquist(G*G1);

%%

G = 1;
s = tf('s');

G2 = (20*(s^2+s+0.5))/(s*(s-1)*(s+10))
figure(2);
nyquist(G*G2);

%%

G = 1;
s = tf('s');

G3 = (s^2+3)/((s+1)^2)
figure(3);
nyquist(G*G3);

%%

G = 1;
s = tf('s');

G4 = (3*(s+1))/(s*(s-10))
figure(4);
nyquist(G*G4);
