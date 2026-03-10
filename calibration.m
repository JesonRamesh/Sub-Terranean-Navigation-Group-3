%% straight line file
load('Training Data/calib2_straight.mat')

% Gyroscope
gyro   = squeeze(out.Sensor_GYRO.signals.values);
time_g = out.Sensor_GYRO.time;


stat_idx = time_g < 60;
gyro_bias = mean(gyro(:, stat_idx), 2);
gyro_corrected = gyro - gyro_bias;
gyro_noise_var = var(gyro_corrected(:, stat_idx), 0, 2)

% Accelerometer

accel  = squeeze(out.Sensor_ACCEL.signals.values);
time_a = out.Sensor_ACCEL.time;


stat_idx_a = time_a < 60;
accel_mean = mean(accel(:, stat_idx_a), 2);
g_measured = norm(accel_mean);
g_true = 9.80665;
scale_factor = g_true / g_measured;
accel_noise_var = var(accel(:, stat_idx_a), 0, 2);

% ToF
tof1 = squeeze(out.Sensor_ToF1.signals.values);
time_t = out.Sensor_ToF1.time;
tof1_dist   = tof1(:, 1);
tof1_status = tof1(:, 4);
valid = tof1_status == 0;
stat_idx_t = time_t < 60;
tof1_stat  = tof1_dist(stat_idx_t & valid);

tof1_mean = mean(tof1_stat);
tof1_var  = var(tof1_stat);
tof1_std  = std(tof1_stat);

tof2 = squeeze(out.Sensor_ToF2.signals.values);
tof3 = squeeze(out.Sensor_ToF3.signals.values);

tof2_dist   = tof2(:, 1);
tof2_status = tof2(:, 4);
tof3_dist   = tof3(:, 1);
tof3_status = tof3(:, 4);

valid2 = tof2_status == 0;
valid3 = tof3_status == 0;

stat_idx_t2 = time_t < 60;
tof2_stat = tof2_dist(stat_idx_t & valid2);
tof3_stat = tof3_dist(stat_idx_t & valid3);

tof2_mean = mean(tof2_stat);
tof2_var  = var(tof2_stat);
tof2_std  = std(tof2_stat);

tof3_mean = mean(tof3_stat);
tof3_var  = var(tof3_stat);
tof3_std  = std(tof3_stat);

%% rotate file
load('Training Data/calib1_rotate.mat')

% Magnetometer
mag    = squeeze(out.Sensor_MAG.signals.values);
time_m = out.Sensor_MAG.time;


hard_iron_y = (max(mag(2,:)) + min(mag(2,:))) / 2;
hard_iron_z = (max(mag(3,:)) + min(mag(3,:))) / 2;

mag_corrected = mag;
mag_corrected(2,:) = mag(2,:) - hard_iron_y;
mag_corrected(3,:) = mag(3,:) - hard_iron_z;


amp_y = (max(mag_corrected(2,:)) - min(mag_corrected(2,:))) / 2;
amp_z = (max(mag_corrected(3,:)) - min(mag_corrected(3,:))) / 2;
avg_amp = (amp_y + amp_z) / 2

soft_iron_y = avg_amp / amp_y;
soft_iron_z = avg_amp / amp_z;

mag_corrected(2,:) = mag_corrected(2,:) * soft_iron_y;
mag_corrected(3,:) = mag_corrected(3,:) * soft_iron_z;

%% summary
calib.gyro.bias        = gyro_bias;
calib.gyro.var         = gyro_noise_var;
calib.accel.mean       = accel_mean;
calib.accel.scale      = scale_factor;
calib.accel.var        = accel_noise_var;
calib.mag.hard_iron_y  = hard_iron_y;
calib.mag.hard_iron_z  = hard_iron_z;
calib.mag.soft_iron_y  = soft_iron_y;
calib.mag.soft_iron_z  = soft_iron_z;
calib.tof.mean         = [tof1_mean; tof2_mean; tof3_mean];
calib.tof.var          = [tof1_var;  tof2_var;  tof3_var];

%%
fprintf('=== Calibration Summary ===\n')
fprintf('Gyro bias:      X=%.5f  Y=%.5f  Z=%.5f rad/s\n', calib.gyro.bias)
fprintf('Accel mean:     X=%.5f  Y=%.5f  Z=%.5f m/s^2 (paste into myEKF OFFLINE_ACCEL_BIAS)\n', calib.accel.mean)
fprintf('Accel scale:    %.4f\n', calib.accel.scale)
fprintf('Mag hard iron:  Y=%.3e  Z=%.3e T\n', calib.mag.hard_iron_y, calib.mag.hard_iron_z)
fprintf('Mag soft iron:  Y=%.4f  Z=%.4f (paste into myEKF MAG_SOFT_IRON)\n', calib.mag.soft_iron_y, calib.mag.soft_iron_z)
fprintf('ToF std:        %.4f  %.4f  %.4f m\n', sqrt(calib.tof.var))