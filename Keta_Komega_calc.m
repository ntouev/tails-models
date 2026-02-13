close all;

wc = 11; % actuator bandwidth

G = tf(1, [1/wc 1], 'InputDelay', 0.0);
I = tf(1,[1 0]); % integrator

Gomega = G*I;
Komega = 7;
Geta = feedback(Komega*Gomega, 1)*I;
Keta = 2.6;
step(feedback(Keta*Geta,1));
stepinfo(feedback(Keta*Geta,1));
grid on;

%%
REF_RATE_x = Komega;
REF_ERR_x = 2*Keta*Komega;

%% conversions for rltool
% G1 = ss(Gomega);
% G2 = ss(I);