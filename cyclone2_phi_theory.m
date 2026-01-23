%% 
% clear; 
% close all;

%% log specifics
synthetic = false;

% 20241030_valken_ewoud
% 144
% tranges = [188 210; 326 332; 380 375; 395 416; 460 470]; servo_delay = 1; Vw = 0; azimw = 0;
% 145
% tranges = [360 364; 366 395; 398 401; 407 418; 421 430; 470 505; 514 580; 680 695]; servo_delay = 1; Vw = 0; azimw = 0;

% 20250117_valken_first_succ_manual
% 0254
% tranges = [590 610; 620 630; 790 808; 814 837; 839 880; 930 980]; servo_delay = 1; Vw = 3; azimw = 45;
% 0257
% tranges = [300 304; 306 325; 345 364; 366 407; 480 193; 497 520; 537 542]; servo_delay = 1; Vw = 2; azimw = 45;

% 20260218_cyberzoo_phi
% 0394
% tranges = [180 400]; servo_delay = 7; Vw = 0; azimw = 0; % whole  flight
% tranges = [200 235]; servo_delay = 7; % roll movements
% tranges = [240 285]; servo_delay = 7; Vw = 0; azimw = 0; % pitch movements
% tranges = [365 400]; servo_delay = 13; Vw = 0; azimw = 0; % compound movements
% tranges = [209 212]; servo_delay = 7; % accel y 
% tranges = [232 236]; servo_delay = 7; % gyro z fit
% tranges = [287 295]; servo_delay = 7; Vw = 0; azimw = 0; % gyro z fit

% 20250303_valken_nav
% 0403
% tranges = [550 614; 638 667; 670 710; 757 763]; servo_delay = 1; Vw = 2; azimw = 270;

% 20250307_valken_spiral
% 0418
tranges = [425 462; 466 612; 650 680; 700 735; 784 794; 820 840]; servo_delay = 1; Vw = 4.5; azimw = 160;
% 0420
% tranges = [1002 1006; 1018 1031; 1033 1066; 1070 1072; 1080 1095]; servo_delay = 1; Vw = 3.5; azimw = 160;

% dataset = 'data_cyclone2v1/yaw.mat'; synthetic = true;
% dataset = 'data_cyclone2v1/roll.mat'; synthetic = true;
% dataset = 'data_cyclone2v1/pitch.mat'; synthetic = true;

%%
if synthetic
    %% load synthetic dataset
    load_data = load(dataset);
    
    t = load_data.t_dr;
    gyrof = load_data.gyrof_dr;
    accelf = load_data.accelf_dr;
    omegaf = load_data.omegaf_dr;
    deltaf = load_data.deltaf_dr;
    vf_b = load_data.vf_b_dr;
    vf = load_data.vf_dr;
    gyrofd = load_data.gyrofd_dr;
    
    datarange = 1:length(t);
else
    %% create ac_data struct
    % ac_data = p.aircrafts.data;

    %% datarange and time vectors
    fs = 500; % Hz
    t = ac_data.IMU_GYRO_SCALED.timestamp;
    
    datarange = [];
    for i = 1:size(tranges,1)
        trange = tranges(i,:);

        datarange_start = find(ac_data.IMU_GYRO_SCALED.timestamp > trange(1), 1, 'first') - 1;
        datarange_end = find(ac_data.IMU_GYRO_SCALED.timestamp > trange(2), 1, 'first') - 1;
        
        datarange = [datarange datarange_start:datarange_end];
    end
    
    %% quat, velocity
    % quat logging at 100 Hz
    quat = double([ac_data.AHRS_REF_QUAT.body_qi ac_data.AHRS_REF_QUAT.body_qx ac_data.AHRS_REF_QUAT.body_qy ac_data.AHRS_REF_QUAT.body_qz]);
    refquat = double([ac_data.AHRS_REF_QUAT.ref_qi ac_data.AHRS_REF_QUAT.ref_qx ac_data.AHRS_REF_QUAT.ref_qy ac_data.AHRS_REF_QUAT.ref_qz]);
    [refquat_t,irefquat_t,~] = unique(ac_data.AHRS_REF_QUAT.timestamp);
    quat = quat(irefquat_t,:);
    refquat = refquat(irefquat_t,:);
    
    v_i = [ac_data.ROTORCRAFT_FP.vnorth_alt, ac_data.ROTORCRAFT_FP.veast_alt, -ac_data.ROTORCRAFT_FP.vup_alt];
    v_wind = repmat([-Vw*cosd(azimw) -Vw*sind(azimw) 0], length(v_i), 1);
    v_i = v_i - v_wind;

    % interpolate to match gyro's measurement frequency (500 Hz)
    quat = interp1(refquat_t, quat, t, "linear", "extrap");
    v_i = interp1(ac_data.ROTORCRAFT_FP.timestamp, v_i, t, "linear", "extrap");
    
    for i = 1:length(t)
        v_b(i,:) = quatrotate(quatnormalize(quat(i,:)), v_i(i,:));
    end
    
    %% IMU measurements
    % gyro, accel, and FBW measurements already at 500 Hz (servo feedback not really)
    gyro = [ac_data.IMU_GYRO_SCALED.gp_alt ac_data.IMU_GYRO_SCALED.gq_alt ac_data.IMU_GYRO_SCALED.gr_alt]/180*pi;
    accel = [ac_data.IMU_ACCEL_SCALED.ax_alt ac_data.IMU_ACCEL_SCALED.ay_alt ac_data.IMU_ACCEL_SCALED.az_alt];
    
    %% rpm and deflections
    rpm1 = double(ac_data.SERIAL_ACT_T4_IN.motor_1_rpm);
    rpm2 = double(ac_data.SERIAL_ACT_T4_IN.motor_2_rpm);
    pos1 = double(ac_data.SERIAL_ACT_T4_IN.rotor_1_az_angle);
    pos2 = double(ac_data.SERIAL_ACT_T4_IN.rotor_2_az_angle);
    rpm = [rpm1, rpm2];
    pos = [pos1, pos2];
    % actuator feedback at 500 Hz but not necessarily synched with quat
    % logging. Thus inter just to ensure same vector size
    rpm = interp1(ac_data.SERIAL_ACT_T4_IN.timestamp, rpm, t, "linear", "extrap");
    pos = interp1(ac_data.SERIAL_ACT_T4_IN.timestamp, pos, t, "linear", "extrap");
    
    % convert to SI
    omega = (2*pi/60) * rpm;
    delta = deg2rad(pos/100);
    % is there a comm delay? Then remove it here
    delta = [delta(servo_delay:end,:); ones(servo_delay-1,2)*delta(end,2)];
    % only for cyclone2v1
    [omega(:,1), omega(:,2)] = deal(omega(:,2), omega(:,1)); % swapped escs
    delta(:,2) = delta(:,2) - deg2rad(5); delta(:,1) = delta(:,1) + deg2rad(1); % fix zero moment deflections
    % switch to phi theory conventions
    delta(:,2) = -delta(:,2); % matters
    omega(:,1) = -omega(:,1); % does not matter (omegas are squared)
    
    % kf = 0.00000735; Sp = 2*0.0324; ro = 1.225;
    % ps = sqrt(.^2 + ((kf*omega(:,1).^2)/(ro*Sp))); % WRONG CALCULATION! USE THETA
    
    %% Rotate vectors to phi-theory frame (x nose)
    % pitch +90; could be done with quatrotate but is defined here explicitly
    v_b = [-v_b(:,3), v_b(:,2), v_b(:,1)];
    gyro = [-gyro(:,3), gyro(:,2), gyro(:,1)];
    accel = [-accel(:,3), accel(:,2), accel(:,1)];
    
    %% filter with Butterworth
    filter_freq = 3;
    [b, a] = butter(2,filter_freq/(fs/2));
    
    gyrof = filter(b,a,gyro,get_ic(b,a,gyro(1,:)));
    accelf= filter(b,a,accel,get_ic(b,a,accel(1,:)));
    omegaf= filter(b,a,omega,get_ic(b,a,omega(1,:)));
    deltaf = filter(b,a,delta,get_ic(b,a,delta(1,:)));
    vf_b = filter(b,a,v_b,get_ic(b,a,v_b(1,:)));
    
    v = sqrt(v_b(:,1).^2 + v_b(:,2).^2 + v_b(:,3).^2);
    vf = filter(b,a,v,get_ic(b,a,v(1,:)));
    % psf = filter(b,a,ps,get_ic(b,a,ps(1,:)));
    
    %% filter with Notch
    % % this was tested but not vey well. If planning on using it
    % % further debugging is needed to see if this is a proper implementation.
    % [b, a] = designNotchPeakIIR(response="notch", CenterFrequency=122/(fs/2), QualityFactor=1.5);
    % 
    % gyrof = filter(b,a,gyrof,get_ic(b,a,gyrof(1,:)));
    % accelf= filter(b,a,accelf,get_ic(b,a,accelf(1,:)));
    % omegaf= filter(b,a,omegaf,get_ic(b,a,omegaf(1,:)));
    % deltaf = filter(b,a,deltaf,get_ic(b,a,deltaf(1,:)));
    % vf_b = filter(b,a,vf_b,get_ic(b,a,vf_b(1,:)));
    % vf = filter(b,a,vf,get_ic(b,a,vf(1,:)));
    
    %% find derivatives
    accelfd = [zeros(1,3); diff(accelf,1)]*fs;
    gyrofd = [zeros(1,3); diff(gyrof,1)]*fs;
    gyrofdd = [zeros(1,3); diff(gyrofd,1)]*fs;
    deltafd = [zeros(1,2); diff(deltaf,1)]*fs;
    deltafdd = [zeros(1,2); diff(deltafd,1)]*fs;
    vfd = [zeros(1,1); diff(vf,1)]*fs;
    vfd_b = [zeros(1,3); diff(vf_b,1)]*fs;
    omegafd = [zeros(1,2); diff(omegaf,1)]*fs;

end

%% fit Accel 1
Xaccel1 = [omegaf(datarange,1).^2 + omegaf(datarange,2).^2, ...
           vf(datarange).*vf_b(datarange,1)];

% Xaccel1 = [2*omegaf(datarange,1).*omegafd(datarange,1) + 2*omegaf(datarange,2).*omegafd(datarange,2), ...
%            vfd(datarange).*vf_b(datarange,1) + vf(datarange).*vfd_b(datarange,1)];

Yaccel1 = accelf(datarange,1);

mdl_accel1 = fitlm(Xaccel1, Yaccel1, "linear", 'Intercept', false);

%% fit Accel 2
Xaccel2 = [vf(datarange).*vf_b(datarange,2)];

Yaccel2 = accelf(datarange,2);

mdl_accel2 = fitlm(Xaccel2, Yaccel2, "linear", 'Intercept', false);

%% fit Accel 3
% Xaccel3 = [omegaf(datarange,1).^2 + omegaf(datarange,2).^2, ...
%            vf(datarange).*vf_b(datarange,1), ...
%            vf(datarange).*vf_b(datarange,3), ...
%            (deltaf(datarange,1) + deltaf(datarange,2)).*vf(datarange).*vf_b(datarange,1), ...
%            deltaf(datarange,1).*omegaf(datarange,1).^2 + deltaf(datarange,2).*omegaf(datarange,2).^2];

Xaccel3 = [2*omegaf(datarange,1).*omegafd(datarange,1) + 2*omegaf(datarange,2).*omegafd(datarange,2), ...
           vfd(datarange).*vf_b(datarange,1) + vf(datarange).*vfd_b(datarange,1), ...
           vfd(datarange).*vf_b(datarange,3) + vf(datarange).*vfd_b(datarange,3), ...
           (deltafd(datarange,1) + deltafd(datarange,2)).*vf(datarange).*vf_b(datarange,1) + ...
               (deltaf(datarange,1) + deltaf(datarange,2)).*(vfd(datarange).*vf_b(datarange,1) + vf(datarange).*vfd_b(datarange,1)), ...
           deltafd(datarange,1).*omegaf(datarange,1).^2 + 2*deltaf(datarange,1).*omegaf(datarange,1).*omegafd(datarange,1) + ...
               deltafd(datarange,2).*omegaf(datarange,2).^2 + 2*deltaf(datarange,2).*omegaf(datarange,2).*omegafd(datarange,2)];

Yaccel3 = accelfd(datarange,3);

mdl_accel3 = fitlm(Xaccel3, Yaccel3, "linear", 'Intercept', false);

%% fit Ang Accel 1
Xang1 = [2*omegaf(datarange,1).*omegafd(datarange,1) - 2*omegaf(datarange,2).*omegafd(datarange,2), ... omega1^2 - omega2^2
         deltafd(datarange,1).*omegaf(datarange,1).^2 + 2*deltaf(datarange,1).*omegaf(datarange,1).*omegafd(datarange,1) - ...
             (deltafd(datarange,2).*omegaf(datarange,2).^2 + 2*deltaf(datarange,2).*omegaf(datarange,2).*omegafd(datarange,2)), ... % delta1*omega1^2 - delta2*omega2^2
         (deltafd(datarange,1) - deltafd(datarange,2)).*vf(datarange).*vf_b(datarange,1) + ...
             (deltaf(datarange,1) - deltaf(datarange,2)).*(vfd(datarange).*vf_b(datarange,1) + vf(datarange).*vfd_b(datarange,1)), ... % (delta1 - delta2)*v*vb1
         gyrofd(datarange,2).*gyrof(datarange,3) + gyrof(datarange,2).*gyrofd(datarange,3)]; % wb2*wb3

Yang1 = gyrofdd(datarange,1);

mdl_ang1 = fitlm(Xang1, Yang1, "linear", 'Intercept', false);

%% fit Ang Accel 2
Xang2 = [vfd(datarange).*vf_b(datarange,1) + vf(datarange).*vfd_b(datarange,1), ... % v*vb1
         vfd(datarange).*vf_b(datarange,3) + vf(datarange).*vfd_b(datarange,3), ... % v*vb3
         deltafdd(datarange,1) + deltafdd(datarange,2), ... % deltad1 + deltad2
         2*omegaf(datarange,1).*omegafd(datarange,1) + 2*omegaf(datarange,2).*omegafd(datarange,2), ... % omega1^2 + omega2^2
         deltafd(datarange,1).*omegaf(datarange,1).^2 + 2*deltaf(datarange,1).*omegaf(datarange,1).*omegafd(datarange,1) + ...
             deltafd(datarange,2).*omegaf(datarange,2).^2 + 2*deltaf(datarange,2).*omegaf(datarange,2).*omegafd(datarange,2), ... % delta1*omega1^2 + delta2*omega2^2
         (deltafd(datarange,1) + deltafd(datarange,2)).*vf(datarange).*vf_b(datarange,1) + ...
             (deltaf(datarange,1) + deltaf(datarange,2)).*(vfd(datarange).*vf_b(datarange,1) + vf(datarange).*vfd_b(datarange,1)), ... % (delta1 + delta2)*v*vb1
         gyrofd(datarange,1).*gyrof(datarange,3) + gyrof(datarange,1).*gyrofd(datarange,3)]; % wb1*wb3

Yang2 = gyrofdd(datarange,2);

mdl_ang2 = fitlm(Xang2, Yang2, "linear", 'Intercept', false);

%% fit Ang Accel 3
Xang3 = [vfd(datarange).*vf_b(datarange,2) + vf(datarange).*vfd_b(datarange,2), ... % v*vb2
         2*omegaf(datarange,1).*omegafd(datarange,1) - 2*omegaf(datarange,2).*omegafd(datarange,2), ... omega1^2 - omega2^2
         vfd(datarange).*gyrof(datarange,1) + vf(datarange).*gyrofd(datarange,1), ... % v*wb1
         gyrofd(datarange,1).*gyrof(datarange,2) + gyrof(datarange,1).*gyrofd(datarange,2)]; % wb1*wb2

Yang3 = gyrofdd(datarange,3);

mdl_ang3 = fitlm(Xang3, Yang3, "linear", 'Intercept', false);

%% Plots
fprintf('accel1 R^2: %.2f\t', mdl_accel1.Rsquared.Ordinary);
fprintf('Coeff: '); fprintf('%.8f ', mdl_accel1.Coefficients.Estimate); fprintf('\n'); 
fprintf('accel2 R^2: %.2f\t', mdl_accel2.Rsquared.Ordinary);
fprintf('Coeff: '); fprintf('%.8f ', mdl_accel2.Coefficients.Estimate); fprintf('\n'); 
fprintf('accel3 R^2: %.2f\t', mdl_accel3.Rsquared.Ordinary);
fprintf('Coeff: '); fprintf('%.8f ', mdl_accel3.Coefficients.Estimate); fprintf('\n'); 
fprintf('ang1   R^2: %.2f\t', mdl_ang1.Rsquared.Ordinary);
fprintf('Coeff: '); fprintf('%.8f ', mdl_ang1.Coefficients.Estimate); fprintf('\n'); 
fprintf('ang2   R^2: %.2f\t', mdl_ang2.Rsquared.Ordinary);
fprintf('Coeff: '); fprintf('%.8f ', mdl_ang2.Coefficients.Estimate); fprintf('\n'); 
fprintf('ang3   R^2: %.2f\t', mdl_ang3.Rsquared.Ordinary);
fprintf('Coeff: '); fprintf('%.8f ', mdl_ang3.Coefficients.Estimate); fprintf('\n'); 

figure('Name','Phi theory fit');
tiledlayout(3, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

ax1 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), Yaccel1, '.', MarkerEdgeColor='b', DisplayName="Real data");
plot(t(datarange), mdl_accel1.Fitted, '.', MarkerEdgeColor='r', DisplayName="Interpolated data");
xlabel('t[sec]');
ylabel('[m/s^2]');
title('Accel 1 (AccelZ in pprz frame)');
legend('show');
hold off;

ax2 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), Yang1, '.', MarkerEdgeColor='b', DisplayName="Real data", LineWidth=1.5);
plot(t(datarange), mdl_ang1.Fitted, '.', MarkerEdgeColor='r', DisplayName="Interpolated data", LineWidth=1.5);
xlabel('t[sec]');
ylabel('[rad/s^2]');
title('Angular Accel 1 (Angular AccelZ in pprz frame - HOVER YAW)');
legend('show');
hold off;

ax3 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), deltaf(datarange,1), MarkerEdgeColor='b', DisplayName="delta 1", LineWidth=1.5);
plot(t(datarange), deltaf(datarange,2), MarkerEdgeColor='r', DisplayName="delta 2", LineWidth=1.5);
xlabel('t[sec]');
ylabel('[rad]');
title('Elevon Deflection');
legend('show');
hold off;

ax4 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), Yaccel2, '.', MarkerEdgeColor='b', DisplayName="Real data", LineWidth=1.5);
plot(t(datarange), mdl_accel2.Fitted, '.', MarkerEdgeColor='r', DisplayName="Interpolated data", LineWidth=1.5);
xlabel('t[sec]');
ylabel('[m/s^2]');
title('Accel 2');
legend('show');
hold off;

ax5 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), Yang2, '.', MarkerEdgeColor='b', DisplayName="Real data", LineWidth=1.5);
plot(t(datarange), mdl_ang2.Fitted, '.', MarkerEdgeColor='r', DisplayName="Interpolated data", LineWidth=1.5);
xlabel('t[sec]');
ylabel('[rad/s^2]');
title('Angular Accel 2');
legend('show');
hold off;

ax6 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), omegaf(datarange,1), MarkerEdgeColor='b', DisplayName="omega 1", LineWidth=1.5);
plot(t(datarange), omegaf(datarange,2), MarkerEdgeColor='r', DisplayName="omega 2", LineWidth=1.5);
xlabel('t[sec]');
ylabel('[rad/sec]');
title('Prop Speed');
legend('show');
hold off;

ax7 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), Yaccel3, '.', MarkerEdgeColor='b', DisplayName="Real data", LineWidth=1.5);
plot(t(datarange), mdl_accel3.Fitted, '.', MarkerEdgeColor='r', DisplayName="Interpolated data", LineWidth=1.5);
xlabel('t[sec]');
ylabel('[m/s^2]');
title('Accel 3');
legend('show');
hold off;

ax8 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), Yang3, '.', MarkerEdgeColor='b', DisplayName="Real data", LineWidth=1.5);
plot(t(datarange), mdl_ang3.Fitted, '.', MarkerEdgeColor='r', DisplayName="Interpolated data", LineWidth=1.5);
xlabel('t[sec]');
ylabel('[rad/s^2]');
title('Angular Accel 3');
legend('show');
hold off;

ax9 = nexttile;
hold on; grid on; zoom on;
plot(t(datarange), vf_b(datarange,1), MarkerEdgeColor='b', DisplayName="Vb1", LineWidth=1.5);
plot(t(datarange), vf_b(datarange,2), MarkerEdgeColor='b', DisplayName="Vb2", LineWidth=1.5);
plot(t(datarange), vf_b(datarange,3), MarkerEdgeColor='b', DisplayName="Vb3", LineWidth=1.5);
plot(t(datarange), vf(datarange), MarkerEdgeColor='b', DisplayName="|Vb|", LineWidth=1.5);
xlabel('t[sec]');
ylabel('[m/s]');
title('Body Velocity');
legend('show');
hold off;

linkaxes([ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8,ax9],'x');

%% Save signals for simulink model validation signals should be given in pprz frame (x belly)
% simin_uf = [t(datarange) omegaf(datarange,1) omegaf(datarange,2) deltaf(datarange,1) deltaf(datarange,2)];
% simin_vfb = [t(datarange) vf_b(datarange,3) vf_b(datarange,2) -vf_b(datarange,1)];
% simin_wfb = [t(datarange) gyrof(datarange,3) gyrof(datarange,2) -gyrof(datarange,1)];

%% save model
% save('mdl_accel1.mat', 'mdl_accel1');
% save('mdl_accel2.mat', 'mdl_accel2');
% save('mdl_accel3.mat', 'mdl_accel3');
% save('mdl_ang1.mat', 'mdl_ang1');
% save('mdl_ang2.mat', 'mdl_ang2');
% save('mdl_ang3.mat', 'mdl_ang3');