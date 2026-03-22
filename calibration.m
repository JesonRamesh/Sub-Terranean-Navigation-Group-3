%% straight line file
load('calib1_rotate.mat') % Make sure path is correct

% Define a strictly STATIONARY time window (e.g., first 2 seconds)
% Adjust this based on looking at a plot of your ToF or Accel data!
static_time_limit = 2.0; 

% Gyroscope
gyro   = squeeze(out.Sensor_GYRO.signals.values);
time_g = out.Sensor_GYRO.time;
stat_idx = time_g < static_time_limit;

gyro_bias = mean(gyro(:, stat_idx), 2);
gyro_corrected = gyro - gyro_bias;
gyro_noise_var = var(gyro_corrected(:, stat_idx), 0, 2);

% Accelerometer
accel  = squeeze(out.Sensor_ACCEL.signals.values);
time_a = out.Sensor_ACCEL.time;
stat_idx_a = time_a < static_time_limit;

% The mean during the stationary period is our exact bias for X and Y!
accel_mean = mean(accel(:, stat_idx_a), 2); 
accel_noise_var = var(accel(:, stat_idx_a), 0, 2);

% ToF
tof1 = squeeze(out.Sensor_ToF1.signals.values);
time_t = out.Sensor_ToF1.time;
tof1_dist   = tof1(:, 1);
tof1_status = tof1(:, 4);
valid = tof1_status == 0;
stat_idx_t = time_t < static_time_limit;

% Now we are only taking the variance while the robot is sitting still!
tof1_stat = tof1_dist(stat_idx_t & valid);
tof1_mean = mean(tof1_stat);
tof1_var  = var(tof1_stat);

tof2 = squeeze(out.Sensor_ToF2.signals.values);
tof3 = squeeze(out.Sensor_ToF3.signals.values);
tof2_dist   = tof2(:, 1);
tof2_status = tof2(:, 4);
tof3_dist   = tof3(:, 1);
tof3_status = tof3(:, 4);

valid2 = tof2_status == 0;
valid3 = tof3_status == 0;

tof2_stat = tof2_dist(stat_idx_t & valid2);
tof3_stat = tof3_dist(stat_idx_t & valid3);

tof2_var  = var(tof2_stat);
tof3_var  = var(tof3_stat);

%% rotate file
load('calib1_rotate.mat') % Make sure path is correct
% Magnetometer (Your code here was perfect!)
mag    = squeeze(out.Sensor_MAG.signals.values);
time_m = out.Sensor_MAG.time;
hard_iron_y = (max(mag(2,:)) + min(mag(2,:))) / 2;
hard_iron_z = (max(mag(3,:)) + min(mag(3,:))) / 2;
mag_corrected = mag;
mag_corrected(2,:) = mag(2,:) - hard_iron_y;
mag_corrected(3,:) = mag(3,:) - hard_iron_z;
amp_y = (max(mag_corrected(2,:)) - min(mag_corrected(2,:))) / 2;
amp_z = (max(mag_corrected(3,:)) - min(mag_corrected(3,:))) / 2;
avg_amp = (amp_y + amp_z) / 2;
soft_iron_y = avg_amp / amp_y;
soft_iron_z = avg_amp / amp_z;

%% summary
fprintf('=== Calibration Summary ===\n')
fprintf('Gyro bias:      X=%.5f  Y=%.5f  Z=%.5f rad/s\n', gyro_bias)
fprintf('Gyro var:       X=%.5f  Y=%.5f  Z=%.5f\n', gyro_noise_var)
fprintf('Accel bias:     X=%.5f  Y=%.5f  Z=%.5f m/s^2 \n', accel_mean)
fprintf('Accel var:      X=%.5f  Y=%.5f  Z=%.5f\n', accel_noise_var)
fprintf('Mag hard iron:  Y=%.3e  Z=%.3e T\n', hard_iron_y, hard_iron_z)
fprintf('Mag soft iron:  Y=%.4f  Z=%.4f \n', soft_iron_y, soft_iron_z)
fprintf('ToF std:        %.4f  %.4f  %.4f m\n', sqrt(tof1_var), sqrt(tof2_var), sqrt(tof3_var))