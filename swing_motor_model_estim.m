clear; close all;

motor_data = readtable('~/LOGS/swing/motor_test/1000to5000pprz.csv');
t5000 = motor_data.timestamp - motor_data.timestamp(1);
norm_omega5000 = (motor_data.rpm - min(motor_data.rpm)) / (max(motor_data.rpm) - min(motor_data.rpm));
omega5000 = motor_data.rpm;

motor_data = readtable('~/LOGS/swing/motor_test/1000to7000pprz.csv');
t7000 = motor_data.timestamp - motor_data.timestamp(1);
norm_omega7000 = (motor_data.rpm - min(motor_data.rpm)) / (max(motor_data.rpm) - min(motor_data.rpm));
omega7000 = motor_data.rpm;

motor_data = readtable('~/LOGS/swing/motor_test/1000to9600pprz.csv');
t9600 = motor_data.timestamp - motor_data.timestamp(1);
norm_omega9600 = (motor_data.rpm - min(motor_data.rpm)) / (max(motor_data.rpm) - min(motor_data.rpm));
omega9600 = motor_data.rpm;

datarange = 1:300;
figure('Name', 'Normalized step responses');
plot(t5000(datarange), norm_omega5000(datarange)); hold on;
plot(t7000(datarange), norm_omega7000(datarange));
plot(t9600(datarange), norm_omega9600(datarange));
yline(0.67, ':k');
xlabel('msec'); ylabel('norm rpm');
legend('1000 to 5000 pprz','1000 to 7000 pprz','1000 to 9600 pprz');
% 
% figure('Name', 'Step responses');
% plot(t5000(datarange), omega5000(datarange)); hold on;
% plot(t7000(datarange), omega7000(datarange));
% plot(t9600(datarange), omega9600(datarange));
% xlabel('msec'); ylabel('rpm');
% legend('1000 to 5000 pprz','1000 to 7000 pprz','1000 to 9600 pprz');

cmd = [3000; 4000; 5000; 6000; 7000; 8000; 9000; 9600];
rpm = [12900; 15100; 17000; 18890; 21010; 23100; 24950; 25900];
thrust = 9.81*[8; 12; 15; 19; 23; 29; 34; 37]/1000 ;
omega = rpm*2*pi/60;

k_omega = (omega.^2) \ thrust;

figure('Name','thrust constant');
plot(omega, thrust); hold on;
plot(omega, k_omega*omega.^2, ':r', LineWidth=2);
xlabel('rad/sec'); ylabel('N');
legend('real','estimated');