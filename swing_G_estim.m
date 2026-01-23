%% 
% clear; 
% close all;

%% log specifics

% log name here
trange = [591.5 616];

ac_data = p.aircrafts.data;

%% datarange and time vectors
fs = 500; % Hz
t = ac_data.IMU_GYRO_SCALED.timestamp;

datarange1 = find(ac_data.IMU_GYRO_SCALED.timestamp>trange(1),1,'first')-1;
datarange2 = find(ac_data.IMU_GYRO_SCALED.timestamp>trange(2),1,'first')-1;
datarange = datarange1:datarange2;

%% get euler angles (zxy order) and airspeed

% quat logging at 100 Hz
quat = double([ac_data.AHRS_REF_QUAT.body_qi ac_data.AHRS_REF_QUAT.body_qx ac_data.AHRS_REF_QUAT.body_qy ac_data.AHRS_REF_QUAT.body_qz]);
refquat = double([ac_data.AHRS_REF_QUAT.ref_qi ac_data.AHRS_REF_QUAT.ref_qx ac_data.AHRS_REF_QUAT.ref_qy ac_data.AHRS_REF_QUAT.ref_qz]);
[refquat_t,irefquat_t,~] = unique(ac_data.AHRS_REF_QUAT.timestamp);
quat = quat(irefquat_t,:);
refquat = refquat(irefquat_t,:);
% [psi, phi, theta] = quat2angle(quat,'ZXY');
% [refpsi, refphi, reftheta] = quat2angle(refquat,'ZXY');

% interpolate quat and airspeed to match gyro's measurement frequency (500 Hz)
quat = interp1(refquat_t, quat, t, "linear", "extrap");
V = interp1(ac_data.AIR_DATA.timestamp, ac_data.AIR_DATA.airspeed, t, "linear", "extrap");

[~, ~, theta_dr] = quat2angle(quat(datarange,:),'ZXY');

%% 
% gyro, accel, and FBW measurements already at 500 Hz (servo feedback not really)
gyro = [ac_data.IMU_GYRO_SCALED.gp_alt ac_data.IMU_GYRO_SCALED.gq_alt ac_data.IMU_GYRO_SCALED.gr_alt]/180*pi;
accel = [ac_data.IMU_ACCEL_SCALED.ax_alt ac_data.IMU_ACCEL_SCALED.ay_alt ac_data.IMU_ACCEL_SCALED.az_alt];

%%
% rpm1 = double(ac_data.SERIAL_ACT_T4_IN.motor_1_rpm);
% rpm2 = double(ac_data.SERIAL_ACT_T4_IN.motor_2_rpm);

cmd_mot1 = double(ac_data.SERIAL_ACT_T4_OUT.motor_1_dshot_cmd(1:end));
cmd_mot2 = double(ac_data.SERIAL_ACT_T4_OUT.motor_2_dshot_cmd(1:end));
time = t(1):0.002:(t(1) + (length(t)-1)*0.002);
if length(time) ~= length(cmd_mot1)
    cmd_mot1 = cmd_mot1(1:end-1);
    cmd_mot2 = cmd_mot2(1:end-1);
end

G1 = tf(1, [1/19.92 1]);
rpm1 = lsim(G1, cmd_mot1, time);
rpm2 = lsim(G1, cmd_mot2, time);

pos1 = double(ac_data.SERIAL_ACT_T4_IN.rotor_1_az_angle);
pos2 = double(ac_data.SERIAL_ACT_T4_IN.rotor_2_az_angle);
% cmd_servo1 = double(ac_data.SERIAL_ACT_T4_OUT.rotor_1_az_angle_cmd(1:end-1));
% cmd_servo2 = double(ac_data.SERIAL_ACT_T4_OUT.rotor_2_az_angle_cmd(1:end-1));
% time = t(1):0.002:(t(1) + (length(t)-1)*0.002);
% G1d = tf(1, [1/44.24 1], 'InputDelay', 0.017);
% pos1 = lsim(G1d, cmd_servo1, time);
% pos2 = lsim(G1d, cmd_servo2, time);

% convert to pprz scale
rpm = [(rpm1-0)*(9600-0)/(2000-0) + 0, (rpm2-0)*(9600-0)/(2000-0) + 0];
delta = [(pos1+4000)*(9600+9600)/(4000+4000) - 9600 (pos2+4000)*(9600+9600)/(4000+4000) - 9600]; % delta1++ (down), delta2-- (down) => pitch down
% is there a comm delay? Then remove it here
delta = [delta(servo_delay:end,:); ones(servo_delay-1,2)*delta(end,2)];

%% filter with Butterworth (+ Notch???)
% Butterworth
filter_freq = 5;
[b, a] = butter(2,filter_freq/(fs/2));

gyro_filt = filter(b,a,gyro,get_ic(b,a,gyro(1,:)));
accel_filt = filter(b,a,accel,get_ic(b,a,accel(1,:)));
theta_dr_filt = filter(b,a,theta_dr,get_ic(b,a,theta_dr(1,:)));
rpm_filt = filter(b,a,rpm,get_ic(b,a,rpm(1,:)));
delta_filt = filter(b,a,delta,get_ic(b,a,delta(1,:)));
V_filt = filter(b,a,V,get_ic(b,a,V(1,:)));

% % Notch filter; this was tested but not vey well. If planning on using it
% % further debugging is needed to see if this is a proper implementation.
% [b, a] = designNotchPeakIIR(response="notch", CenterFrequency=122/(fs/2), QualityFactor=1.5);
% 
% gyro_filt = filter(b,a,gyro_filt,get_ic(b,a,gyro_filt(1,:)));
% accel_filt = filter(b,a,accel_filt,get_ic(b,a,accel_filt(1,:)));
% theta_dr_filt = filter(b,a,theta_dr_filt,get_ic(b,a,theta_dr_filt(1,:)));
% rpm_filt = filter(b,a,rpm_filt,get_ic(b,a,rpm_filt(1,:)));
% delta_filt = filter(b,a,delta_filt,get_ic(b,a,delta_filt(1,:)));
% V_filt = filter(b,a,V_filt,get_ic(b,a,V_filt(1,:)));

%% find derivatives of filtered values
gyro_filtd = [zeros(1,3); diff(gyro_filt,1)]*fs;
gyro_filtdd = [zeros(1,3); diff(gyro_filtd,1)]*fs;
accel_filtd = [zeros(1,3); diff(accel_filt,1)]*fs;
accel_filtdd = [zeros(1,3); diff(accel_filtd,1)]*fs;
theta_dr_filtd = [zeros(1,1); diff(theta_dr_filt,1)]*fs;
rpm_filtd = [zeros(1,2); diff(rpm_filt,1)]*fs;
delta_filtd = [zeros(1,2); diff(delta_filt,1)]*fs;
V_filtd = [zeros(1,1); diff(V_filt,1)]*fs;

%% Roll effectiveness
% can take derivatives to remove bias (constant offset)
output_roll = gyro_filtdd(datarange,1);
% G = const
% inputs_roll = rpm_filtd(datarange,1) - rpm_filtd(datarange,2);

% G(w1) = G*w1 and G(w2) = G*w2 => p_dd = 2*G*(w1_d*w1 - w2_d*w2). In code
% SQUARED_ROLL_EFF = 2*G, so the feature for L.R. is the one below:
inputs_roll = rpm_filtd(datarange,1) .* rpm_filt(datarange,1) - rpm_filtd(datarange,2) .* rpm_filt(datarange,2); 

Groll = inputs_roll\output_roll; % solve Ax = b
disp("GRoll");
disp(Groll);

%% Pitch effectiveness
output_pitch = gyro_filtdd(datarange,2);

if scale_with_V
    % G(V) = A + B*V^2
    inputs_pitch = [delta_filtd(datarange,1) - delta_filtd(datarange,2) ...
                    2 * V_filt(datarange) .* V_filtd(datarange) .* (delta_filt(datarange,1) - delta_filt(datarange,2)) ...
                    + V_filt(datarange).^2 .* (delta_filtd(datarange,1) - delta_filtd(datarange,2))];
elseif scale_with_theta
    % G(theta) = A + B*theta
    inputs_pitch = [delta_filtd(datarange,1) - delta_filtd(datarange,2), ...
                    delta_filtd(datarange,1) .* theta_dr_filt + delta_filt(datarange,1) .* theta_dr_filtd ...
                    - delta_filtd(datarange,2) .* theta_dr_filt - delta_filt(datarange,2) .* theta_dr_filtd];
else
    % G = const
    inputs_pitch = delta_filtd(datarange,1) - delta_filtd(datarange,2);
end

Gpitch = inputs_pitch\output_pitch;

if scale_with_V
    disp("Gpitch A+B*V^2");
    disp(Gpitch);
elseif scale_with_theta
    [Gpitch_rtheta0, Gpitch_rtheta1] = get_G_rtheta_coeff(Gpitch(1), Gpitch(2)); % assumes theta in rad
    Gpitch_rtheta = [Gpitch_rtheta0; Gpitch_rtheta1];
    
    disp("Gpitch_rtheta");
    disp(Gpitch_rtheta);
else 
    disp("Gpitch const");
    disp(Gpitch); 
end

%% Yaw effectiveness
output_yaw = gyro_filtdd(datarange,3);

if scale_with_V
    % G(V) = A + B*V^2
    inputs_yaw = [delta_filtd(datarange,1) + delta_filtd(datarange,2) ...
                    2 * V_filt(datarange) .* V_filtd(datarange) .* (delta_filt(datarange,1) + delta_filt(datarange,2)) ...
                    + V_filt(datarange).^2 .* (delta_filtd(datarange,1) + delta_filtd(datarange,2))];
elseif scale_with_theta   
    % G(theta) = A + B*theta
    inputs_yaw = [delta_filtd(datarange,1) + delta_filtd(datarange,2), ...
                  delta_filtd(datarange,1) .* theta_dr_filt + delta_filt(datarange,1) .* theta_dr_filtd ...
                  + delta_filtd(datarange,2) .* theta_dr_filt + delta_filt(datarange,2) .* theta_dr_filtd];
else
    % G = const
    inputs_yaw = delta_filtd(datarange,1) + delta_filtd(datarange,2);
end

Gyaw = inputs_yaw\output_yaw;

if scale_with_V
    disp("Gyaw A+B*V^2");
    disp(Gyaw);
elseif scale_with_theta
    [Gyaw_rtheta0, Gyaw_rtheta1] = get_G_rtheta_coeff(Gyaw(1), Gyaw(2)); % assumes theta in rad
    Gyaw_rtheta = [Gyaw_rtheta0; Gyaw_rtheta1];

    disp("Gyaw_rtheta");
    disp(Gyaw_rtheta);
else 
    disp("Gyaw const");
    disp(Gyaw); 
end
%% Thrust effectiveness
output_thrust = accel_filtd(datarange,3);
inputs_thrust = rpm_filtd(datarange,1) + rpm_filtd(datarange,2);%, ...
                 % delta_filt(datarange,1), delta_filt(datarange,2)]; % to account for drag induced by large flaps deflections

Gthrust = inputs_thrust\output_thrust;
disp("GThrust");
disp(Gthrust);

%% Plot
figure('Name','Effectiveness fit');
tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

ax1 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), output_roll);
plot(t(datarange), inputs_roll * Groll);
title(sprintf('Roll Effectiveness; (%.8f)', Groll));
hold off;

ax2 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), output_pitch);
plot(t(datarange), inputs_pitch * Gpitch);
if scale_with_V
    title(sprintf('Gpitch(V) = %.4f + %.4f*V^2', Gpitch(1), Gpitch(2)));
elseif scale_with_theta
    title(sprintf('Gpitch rtheta0 = %.4f, Gpitch rtheta1 = %.4f', Gpitch_rtheta(1), Gpitch_rtheta(2)));
else
    title(sprintf('Gpitch = %.4f', Gpitch));
end
hold off;

ax3 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), output_yaw);
plot(t(datarange), inputs_yaw * Gyaw);
if scale_with_V
    title(sprintf('Gyaw(V) = %.4f + %.4f*V^2', Gyaw(1), Gyaw(2)));
elseif scale_with_theta
    title(sprintf('Gyaw rtheta0 = %.4f, Gyaw rtheta1 = %.4f', Gyaw_rtheta(1), Gyaw_rtheta(2)));
else
    title(sprintf('Gyaw = %.4f', Gyaw));
end
hold off;

ax4 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), output_thrust);
plot(t(datarange), inputs_thrust * Gthrust);
title(sprintf('Thrust Effectiveness; (%.4f)', Gthrust));
hold off;

ax5 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), rad2deg(theta_dr_filt), 'LineWidth', 1.5);
plot(t(datarange), V_filt(datarange), 'LineWidth', 1.5);
yline(6, 'r--');
yline(-30, 'b--');
yline(-60, 'b--');
title('Scaling variables');
legend('theta [deg]', 'airspeed [m/s]', '6 m/s', '-30 deg', '-60 deg');
hold off;

ax6 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), delta_filtd(datarange,1));
plot(t(datarange), delta_filtd(datarange,2));
legend('delta1 dot', 'delta 2 dot');
hold off;

linkaxes([ax1,ax2,ax3,ax4,ax5,ax6], 'x');