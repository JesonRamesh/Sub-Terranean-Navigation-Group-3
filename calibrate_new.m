%% calibrate.m — Standalone calibration script
%  Loads calibration .mat files, computes sensor biases/offsets,
%  and produces diagnostic plots.
%
%  NOTE: The values computed here should be hardcoded into myEKF.m.
%        This script is for reference/verification only.
%
%  Expected files:
%    calib1_extracted.mat  — stationary + straight line drive
%    calib2_extracted.mat  — 5 full rotation cycles

clear; clc; close all;

%% ===== LOAD DATA =====
fprintf('=== Loading calibration data ===\n');
c1 = load('calib1_extracted.mat'); % stationary + straight line
c2 = load('calib2_extracted.mat'); % rotation cycles

% Handle nested struct (if saved as workspace with 'out' variable)
if isfield(c1, 'out'), c1 = c1.out; end
if isfield(c2, 'out'), c2 = c2.out; end

%% ===== GYROSCOPE BIAS (from stationary period in calib1) =====
fprintf('\n=== GYROSCOPE CALIBRATION ===\n');

gyro_vals = squeeze(c1.Sensor_GYRO.signals.values);  % collapse 3D -> 2D
gyro_time = c1.Sensor_GYRO.time(:);

% Ensure 3 x N
if size(gyro_vals, 1) ~= 3
    gyro_vals = gyro_vals';
end

% Use first 60 seconds as stationary period
N_gyro = length(gyro_time);
stationary_mask = gyro_time <= 60;

% Skip first 5 samples (zero-padded)
stationary_mask(1:min(5, N_gyro)) = false;

gyro_stationary = gyro_vals(:, stationary_mask);

gyro_bias = mean(gyro_stationary, 2);
gyro_var  = var(gyro_stationary, 0, 2);

fprintf('Gyro bias [rad/s]:    [%.5f, %.5f, %.5f]\n', gyro_bias);
fprintf('Gyro variance:        [%.2e, %.2e, %.2e]\n', gyro_var);

%% ===== VERIFY GYRO YAW AXIS (from calib2 rotation data) =====
fprintf('\n=== GYRO YAW AXIS VERIFICATION (calib2 — 5 rotation cycles) ===\n');

gyro2_vals = squeeze(c2.Sensor_GYRO.signals.values);
gyro2_time = c2.Sensor_GYRO.time(:);
if size(gyro2_vals, 1) ~= 3, gyro2_vals = gyro2_vals'; end

% Integrate each axis to see which accumulates ~5 full rotations
for ax = 1:3
    dt_vec = diff(gyro2_time);
    integrated = cumsum((gyro2_vals(ax, 2:end) - gyro_bias(ax)) .* dt_vec');
    total_rad = integrated(end);
    total_cycles = total_rad / (2*pi);
    fprintf('  Axis %d: total = %.2f rad = %.2f cycles\n', ax, total_rad, total_cycles);
end

%% ===== ACCELEROMETER BIAS (from stationary period in calib1) =====
fprintf('\n=== ACCELEROMETER CALIBRATION ===\n');

accel_vals = squeeze(c1.Sensor_ACCEL.signals.values);
accel_time = c1.Sensor_ACCEL.time(:);
if size(accel_vals, 1) ~= 3, accel_vals = accel_vals'; end

N_accel = length(accel_time);
stat_mask_a = accel_time <= 60;
stat_mask_a(1:min(5, N_accel)) = false;

accel_stationary = accel_vals(:, stat_mask_a);
accel_mean = mean(accel_stationary, 2);
accel_var  = var(accel_stationary, 0, 2);

fprintf('Accel stationary mean [m/s²]: [%.3f, %.3f, %.3f]\n', accel_mean);
fprintf('Accel variance:               [%.2e, %.2e, %.2e]\n', accel_var);
fprintf('  (Row 1 ≈ gravity ~10 m/s², Rows 2,3 = planar biases)\n');

%% ===== MAGNETOMETER HARD-IRON OFFSET (from rotation in calib2) =====
fprintf('\n=== MAGNETOMETER CALIBRATION ===\n');

mag_vals = squeeze(c2.Sensor_MAG.signals.values);
mag_time = c2.Sensor_MAG.time(:);
if size(mag_vals, 1) ~= 3, mag_vals = mag_vals'; end

% Skip zero-padded samples
mag_vals = mag_vals(:, 5:end);

% Hard-iron offset = centre of the ellipse (min+max)/2 for each axis
mag_hard_iron = zeros(3,1);
for ax = 1:3
    mag_hard_iron(ax) = (max(mag_vals(ax,:)) + min(mag_vals(ax,:))) / 2;
end

fprintf('Mag hard-iron offset [T]: [%.2e, %.2e, %.2e]\n', mag_hard_iron);

% Corrected mag
mag_corrected = mag_vals - mag_hard_iron;

%% ===== QUATERNION TO YAW — GROUND TRUTH =====
fprintf('\n=== GROUND TRUTH YAW CHECK ===\n');

gt_rot2 = squeeze(c2.GT_rotation.signals.values);
if size(gt_rot2, 2) ~= 4, gt_rot2 = gt_rot2'; end
% Quaternion [w, x, y, z]
gt_yaw2 = atan2(2*(gt_rot2(:,1).*gt_rot2(:,4) + gt_rot2(:,2).*gt_rot2(:,3)), ...
               1 - 2*(gt_rot2(:,3).^2 + gt_rot2(:,4).^2));
fprintf('GT yaw range: [%.2f, %.2f] rad\n', min(gt_yaw2), max(gt_yaw2));

%% ===== TOF SENSOR CHECK =====
fprintf('\n=== TOF SENSOR INFO ===\n');
for s = 1:3
    tof_name = sprintf('Sensor_ToF%d', s);
    tof_data = squeeze(c1.(tof_name).signals.values);
    if size(tof_data, 2) ~= 4, tof_data = tof_data'; end
    distances = tof_data(:,1);
    statuses  = tof_data(:,4);
    fprintf('  %s: dist range [%.3f, %.3f] m, status values: %s\n', ...
        tof_name, min(distances(distances>0)), max(distances), ...
        mat2str(unique(statuses)));
end

%% ===== YAW OFFSET ESTIMATION =====
fprintf('\n=== YAW OFFSET (sensor frame vs GT frame) ===\n');

% From calib2: compare mag heading to GT yaw
% Mag axes 2,3 are the planar axes (axis 1 = up/gravity)
mag_heading = atan2(mag_corrected(3,:), mag_corrected(2,:));

% Downsample GT to mag rate for comparison
gt_time2 = c2.GT_rotation.time(:);
mag_time2 = c2.Sensor_MAG.time(5:end);
mag_time2 = mag_time2(:);

gt_yaw_interp = interp1(gt_time2, unwrap(gt_yaw2), mag_time2, 'linear', 'extrap');

yaw_diff = wrapToPi(mag_heading' - gt_yaw_interp);
yaw_offset = mean(yaw_diff);
fprintf('Yaw offset (sensor - GT): %.4f rad = %.2f deg\n', yaw_offset, rad2deg(yaw_offset));
fprintf('  (Use ~5.7 deg as per empirical verification)\n');

%% ===== DIAGNOSTIC PLOTS =====
fprintf('\n=== Generating diagnostic plots ===\n');

figure('Name', 'Gyro Stationary Period');
for ax = 1:3
    subplot(3,1,ax);
    plot(gyro_time(stationary_mask), gyro_stationary(ax,:));
    ylabel(sprintf('Axis %d [rad/s]', ax));
    title(sprintf('Gyro Axis %d — Stationary (bias=%.5f)', ax, gyro_bias(ax)));
    grid on;
end
xlabel('Time [s]');

figure('Name', 'Accelerometer Stationary Period');
for ax = 1:3
    subplot(3,1,ax);
    plot(accel_time(stat_mask_a), accel_stationary(ax,:));
    ylabel(sprintf('Axis %d [m/s²]', ax));
    title(sprintf('Accel Axis %d — Stationary (mean=%.3f)', ax, accel_mean(ax)));
    grid on;
end
xlabel('Time [s]');

figure('Name', 'Magnetometer XY (Hard-Iron)');
subplot(1,2,1);
plot(mag_vals(2,:), mag_vals(3,:), '.'); hold on;
plot(mag_hard_iron(2), mag_hard_iron(3), 'rx', 'MarkerSize', 15, 'LineWidth', 2);
title('Raw Mag (Axes 2 vs 3)'); axis equal; grid on;
xlabel('Mag Axis 2 [T]'); ylabel('Mag Axis 3 [T]');

subplot(1,2,2);
plot(mag_corrected(2,:), mag_corrected(3,:), '.'); hold on;
plot(0, 0, 'rx', 'MarkerSize', 15, 'LineWidth', 2);
title('Corrected Mag (Axes 2 vs 3)'); axis equal; grid on;
xlabel('Mag Axis 2 [T]'); ylabel('Mag Axis 3 [T]');

figure('Name', 'Gyro Integration — Rotation Cycles');
dt_vec = diff(gyro2_time);
for ax = 1:3
    subplot(3,1,ax);
    integrated = cumsum((gyro2_vals(ax, 2:end) - gyro_bias(ax)) .* dt_vec');
    plot(gyro2_time(2:end), integrated / (2*pi));
    ylabel('Cycles');
    title(sprintf('Gyro Axis %d integrated (%.1f cycles)', ax, integrated(end)/(2*pi)));
    grid on;
end
xlabel('Time [s]');

%% ===== SUMMARY =====
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║              CALIBRATION SUMMARY                        ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║ Gyro bias [rad/s]:  [%.5f, %.5f, %.5f]     ║\n', gyro_bias);
fprintf('║ Gyro var:           [%.2e, %.2e, %.2e]  ║\n', gyro_var);
fprintf('║ Accel mean [m/s²]:  [%.3f, %.3f, %.3f]          ║\n', accel_mean);
fprintf('║ Accel var:          [%.2e, %.2e, %.2e]  ║\n', accel_var);
fprintf('║ Mag hard-iron [T]:  [%.2e, %.2e, %.2e]  ║\n', mag_hard_iron);
fprintf('║ Yaw offset:         ~5.7 deg (0.0995 rad)              ║\n');
fprintf('║ Gyro yaw axis:      Row 1                               ║\n');
fprintf('║ Accel planar axes:  Rows 2, 3                           ║\n');
fprintf('║ Mag planar axes:    Axes 2, 3 (axis 1 = up)            ║\n');
fprintf('║ ToF mounting:       ToF1=fwd, ToF2=left, ToF3=right    ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');
fprintf('\nHardcode these values into myEKF.m\n');