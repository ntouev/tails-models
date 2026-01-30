%% 
clear; 
close all;

%% log specifics
ac_data = readtable('~/LOGS/swing/20260130_tosca/roll_test.csv');
ac_data(end, :) = [];  % remove the last row
ac_data.time = ac_data.time - ac_data.time(1);
trange = [10 26];

%% t, interpolations, and datarange
fs = 500; % Hz

t = (0:1/fs:ac_data.time(end))';

p = interp1(ac_data.time, ac_data.rate_p, t, "linear", "extrap");

act1 = interp1(ac_data.time, ac_data.actuators_0, t, "linear", "extrap");
act2= interp1(ac_data.time, ac_data.actuators_1, t, "linear", "extrap");
act3= interp1(ac_data.time, ac_data.actuators_2, t, "linear", "extrap");
act4= interp1(ac_data.time, ac_data.actuators_3, t, "linear", "extrap");

datarange1 = find(t>trange(1),1,'first')-1;
datarange2 = find(t>trange(2),1,'first')-1;
datarange = datarange1:datarange2;

%% 1st order actuator dynamics (pprz units)
G1 = tf(1, [1/50 1]);
act1 = lsim(G1, act1, t);
act2 = lsim(G1, act2, t);
act3 = lsim(G1, act3, t);
act4 = lsim(G1, act4, t);

%% filter with Butterworth (+ Notch???)
% Butterworth
filter_freq = 5;
[b, a] = butter(2,filter_freq/(fs/2));

pf = filter(b,a,act1,get_ic(b,a,p(1)));
act1f = filter(b,a,act1,get_ic(b,a,act1(1)));
act2f = filter(b,a,act2,get_ic(b,a,act2(1)));
act3f = filter(b,a,act3,get_ic(b,a,act3(1)));
act4f = filter(b,a,act4,get_ic(b,a,act4(1)));

%% find derivatives of filtered values
pf_d = gradient(p, 1/fs);
act1f_d = gradient(act1f, 1/fs);
act2f_d = gradient(act2f, 1/fs);
act3f_d = gradient(act3f, 1/fs);
act4f_d = gradient(act4f, 1/fs);

%% Roll effectiveness
output_roll = pf_d(datarange);
inputs_roll = (act1f(datarange).^2 + act4f(datarange).^2) - (act2f(datarange).^2 + act3f(datarange).^2)/100000; 

Groll = inputs_roll/output_roll;
disp("GRoll");
disp(Groll);

%% Plot
figure('Name','Effectiveness fit');
hold on; grid on; zoom on;
plot(t(datarange), output_roll);
plot(t(datarange), inputs_roll * Groll);
title(sprintf('Roll Effectiveness; (%.8f)', Groll));
hold off;