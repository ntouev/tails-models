%% 
clear; 
close all;

%% log specifics and fixes
% ac_data = readtable('~/LOGS/swing/20260130_tosca/pitch_test.csv');
% tranges = [11 16.5]; % roll
% tranges = [10 19]; % pitch
% tranges = [8 11; 17 19]; % yaw

% ac_data = readtable('~/LOGS/swing/20260210_G_tuning/roll-pitch-yaw.csv');
% tranges = [13 60];

% ac_data = readtable('~/LOGS/swing/20260210_G_tuning/r-p-thr.csv');
% tranges = [6 35];

ac_data = readtable('~/LOGS/swing/20260210_G_tuning/r-p-y-thr_20_30.csv');
tranges = [4 40];

ac_data(end, :) = [];  % remove the last row
ac_data.timestamp = ac_data.timestamp - ac_data.timestamp(1);
ac_data.acc_z = ac_data.acc_z/1000;

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

accz = interp1(ac_data.timestamp, ac_data.acc_z, t, "linear", "extrap");

cmd_TL = interp1(ac_data.timestamp, ac_data.cmd_TL, t, "linear", "extrap");
cmd_TR = interp1(ac_data.timestamp, ac_data.cmd_TR, t, "linear", "extrap");
cmd_BR = interp1(ac_data.timestamp, ac_data.cmd_BR, t, "linear", "extrap");
cmd_BL = interp1(ac_data.timestamp, ac_data.cmd_BL, t, "linear", "extrap");

%% 1st order actuator dynamics (pprz units)
G1 = tf(1, [1/30 1]);
actTL = lsim(G1, cmd_TL, t);
actTR = lsim(G1, cmd_TR, t);
actBR = lsim(G1, cmd_BR, t);
actBL = lsim(G1, cmd_BL, t);

%% filter with Butterworth (+ Notch???)
% Butterworth
filter_freq = 5;
[b, a] = butter(2,filter_freq/(fs/2));

pf = filter(b, a, p, get_ic(b,a,p(1)));
qf = filter(b, a, q, get_ic(b,a,q(1)));
rf = filter(b, a, r, get_ic(b,a,r(1)));

acczf = filter(b, a, accz, get_ic(b,a,accz(1)));

actTLf = filter(b, a, actTL, get_ic(b,a,actTL(1)));
actTRf = filter(b, a, actTR, get_ic(b,a,actTR(1)));
actBRf = filter(b, a, actBR, get_ic(b,a,actBR(1)));
actBLf = filter(b, a, actBL, get_ic(b,a,actBL(1)));

%% find derivatives of filtered values
pf_d = [zeros(1,1); diff(pf,1)]*fs;
qf_d = [zeros(1,1); diff(qf,1)]*fs;
rf_d = [zeros(1,1); diff(rf,1)]*fs;

pf_dd = [zeros(1,1); diff(pf_d,1)]*fs;
qf_dd = [zeros(1,1); diff(qf_d,1)]*fs;
rf_dd = [zeros(1,1); diff(rf_d,1)]*fs;

acczf_d = [zeros(1,1); diff(acczf,1)]*fs;

actTLf_d = [zeros(1,1); diff(actTLf,1)]*fs;
actTRf_d = [zeros(1,1); diff(actTRf,1)]*fs;
actBRf_d = [zeros(1,1); diff(actBRf,1)]*fs;
actBLf_d = [zeros(1,1); diff(actBLf,1)]*fs;

%% Roll effectiveness
output_roll = pf_dd(datarange);
inputs_roll = [actTLf_d(datarange), actTRf_d(datarange), actBRf_d(datarange), actBLf_d(datarange)]; 

Groll = inputs_roll\output_roll;

%% Pitch effectiveness
output_pitch = qf_dd(datarange);
inputs_pitch = [actTLf_d(datarange), actTRf_d(datarange), actBRf_d(datarange), actBLf_d(datarange)];

Gpitch = inputs_pitch\output_pitch;

%% Yaw effectiveness
output_yaw = rf_dd(datarange);
inputs_yaw = [actTLf_d(datarange), actTRf_d(datarange), actBRf_d(datarange), actBLf_d(datarange)]; 

Gyaw = inputs_yaw\output_yaw;

%% Thrust effectiveness
output_thr = acczf(datarange);
inputs_thr = [actTLf(datarange), actTRf(datarange), actBRf(datarange), actBLf(datarange)]; 

Gthr = inputs_thr\output_thr;

%% Plot
M = [
    Groll(:).';
    Gpitch(:).';
    Gyaw(:).';
    Gthr(:).'
];
fprintf('%.8f %.8f %.8f %.8f\n', M.');

figure('Name','Effectiveness fit');
tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

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

ax4 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), output_thr);
plot(t(datarange), inputs_thr*Gthr);
title('Thrust fit');
hold off;

linkaxes([ax1,ax2,ax3,ax4],'x');