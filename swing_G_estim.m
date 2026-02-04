%% 
clear; 
close all;

%% log specifics
ac_data = readtable('~/LOGS/swing/20260130_tosca/pitch_test.csv'); 
ac_data(end, :) = [];  % remove the last row
ac_data.timestamp = ac_data.timestamp - ac_data.timestamp(1);

% tranges = [11 16.5]; % roll
tranges = [10 19]; % pitch
% tranges = [8 11; 17 19]; % yaw

%% t, datarange
fs = 500;
t = (0:1/fs:ac_data.timestamp(end))';

datarange = [];
for i = 1:size(tranges,1)
    trange = tranges(i,:);

    datarange_start = find(t > trange(1), 1, 'first') - 1;
    datarange_end = find(t > trange(2), 1, 'first') - 1;
    
    datarange = [datarange datarange_start:datarange_end];
end

%% interp
p = interp1(ac_data.timestamp, ac_data.rate_p, t, "linear", "extrap");
q = interp1(ac_data.timestamp, ac_data.rate_q, t, "linear", "extrap");
r = interp1(ac_data.timestamp, ac_data.rate_r, t, "linear", "extrap");

act1 = interp1(ac_data.timestamp, ac_data.actuators_0, t, "linear", "extrap");
act2 = interp1(ac_data.timestamp, ac_data.actuators_1, t, "linear", "extrap");
act3 = interp1(ac_data.timestamp, ac_data.actuators_2, t, "linear", "extrap");
act4 = interp1(ac_data.timestamp, ac_data.actuators_3, t, "linear", "extrap");

%% 1st order actuator dynamics (pprz units)
G1 = tf(1, [1/54 1]);
act1 = lsim(G1, act1, t);
act2 = lsim(G1, act2, t);
act3 = lsim(G1, act3, t);
act4 = lsim(G1, act4, t);

%% filter with Butterworth (+ Notch???)
% Butterworth
filter_freq = 5;
[b, a] = butter(2,filter_freq/(fs/2));

pf = filter(b, a, p, get_ic(b,a,p(1)));
qf = filter(b, a, q, get_ic(b,a,q(1)));
rf = filter(b, a, r, get_ic(b,a,r(1)));

act1f = filter(b, a, act1, get_ic(b,a,act1(1)));
act2f = filter(b, a, act2, get_ic(b,a,act2(1)));
act3f = filter(b, a, act3, get_ic(b,a,act3(1)));
act4f = filter(b, a, act4, get_ic(b,a,act4(1)));

%% find derivatives of filtered values
pf_d = [zeros(1,1); diff(pf,1)]*fs;
% pf_dd = [zeros(1,1); diff(pf_d,1)]*fs;
qf_d = [zeros(1,1); diff(qf,1)]*fs;
% qf_dd = [zeros(1,1); diff(qf_d,1)]*fs;
rf_d = [zeros(1,1); diff(rf,1)]*fs;

act1f_d = [zeros(1,1); diff(act1f,1)]*fs;
act2f_d = [zeros(1,1); diff(act2f,1)]*fs;
act3f_d = [zeros(1,1); diff(act3f,1)]*fs;
act4f_d = [zeros(1,1); diff(act4f,1)]*fs;

%% Roll effectiveness
output_roll = pf_d(datarange);
inputs_roll = [ones(length(t(datarange)),1), ...
               (act3f(datarange) + act4f(datarange)) - (act1f(datarange) + act2f(datarange))]; 
% inputs_roll = 2*act3f(datarange).*act3f_d(datarange) + 2*act4f(datarange).*act4f_d(datarange) - ...
%               (2*act1f(datarange).*act2f_d(datarange) + 2*act1f(datarange).*act2f_d(datarange));

Groll = inputs_roll\output_roll;

%% Pitch effectiveness
output_pitch = qf_d(datarange);
inputs_pitch = [ones(length(t(datarange)),1), ...
                (act2f(datarange) + act4f(datarange)) - (act1f(datarange) + act3f(datarange))]; 

Gpitch = inputs_pitch\output_pitch;

%% Yaw effectiveness
output_yaw = rf_d(datarange);
inputs_yaw = [ones(length(t(datarange)),1), ...
              (act2f(datarange) + act3f(datarange)) - (act1f(datarange) + act4f(datarange))]; 

Gyaw = inputs_yaw\output_yaw;

%% Plot
fprintf('Roll: %.8f\n', Groll(2));
fprintf('Pitch: %.8f\n', Gpitch(2));
fprintf('Yaw: %.8f\n', Gyaw(1));

figure('Name','Effectiveness fit');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

ax1 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), output_roll);
plot(t(datarange), inputs_roll*Groll);
title('p dot fit');
hold off;

ax2 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), output_pitch);
plot(t(datarange), inputs_pitch*Gpitch);
title('q dot fit');
hold off;

ax3 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), output_yaw);
plot(t(datarange), inputs_yaw*Gyaw);
title('r dot fit');
hold off;

linkaxes([ax1,ax2,ax3],'x');