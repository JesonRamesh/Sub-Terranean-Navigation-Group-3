% sensor_diagnostic.m
% Run this script after loading a .mat file from the robot (any task).
% Purpose: understand physical orientation and alignment of all sensors.
%
% =========================================================================
%  LAB SESSION TASKS (2026-03-17)
% =========================================================================
%
%  RECORDING A — Static test (~30 seconds, robot completely still)
%    1. Place robot on the floor, do not touch it
%    2. Start recording in Simulink → wait 30 s → stop
%    3. load('static_test.mat')  then run sensor_diagnostic
%    4. CHECK: Figure 3 — which axis is ~±9.81 m/s²? (that is the gravity/vertical axis)
%    5. CHECK: Figure 4 — are roll and pitch near 0°? (confirms robot is level)
%    6. CHECK: Printed summary — note the bias values for each sensor axis
%
%  RECORDING B — Rotation test (slow 360° manual rotation, ~15 seconds)
%    1. Place robot in open space, start recording
%    2. Slowly rotate robot one full turn by hand
%    3. Stop, load, run sensor_diagnostic
%    4. CHECK: Figure 2 — does axis 1 (gyro) show ~±360°? (confirms yaw axis)
%              If a different axis shows 360°, update EKF: omega = -(gyro(:,N) - bias)
%    5. CHECK: Figure 5 — is the calibrated mag circle round and centred?
%              If not, re-run calibration.m and update hard/soft iron constants in myEKF.m
%    6. CHECK: Figure 6 — do gyro and mag heading agree on the total rotation?
%
%  RECORDING C — Straight line (Task 1 equivalent, PhaseSpace must be running)
%    1. Drive robot straight forward ~1.5 m and back to start
%    2. load, run sensor_diagnostic, then run test_ekf
%    3. CHECK: Figure 8 — does ToF2+ToF3 ≈ 3.36 m?
%              If not, measure arena Y dimension with tape, update arena_bounds in myEKF.m:
%              arena_bounds = struct('x_max',?,'x_min',?,'y_max',?,'y_min',?)
%    4. CHECK: Figure 7 — ToF2 decreases and ToF3 increases smoothly as robot moves +Y
%    5. CHECK: test_ekf RMSE — should be <0.05 m using fix-task2-from-helitha branch
%
%  RECORDING D — Circuit (Task 2, PhaseSpace must be running)
%    1. Drive robot through the circuit course
%    2. load, run test_ekf (or test_ekf_task2 if available)
%    3. CHECK: RMSE — goal is <0.1 m
%    4. If RMSE is large: run sensor_diagnostic, check Figure 6 during turns
%       (gyro and mag should still agree during turns — if not, gyro sign or bias is wrong)
%
%  KEY THINGS TO WRITE DOWN IN THE LAB:
%    - Actual arena X and Y dimensions (measure with tape)
%    - Which gyro axis responds to turning (should be axis 1)
%    - The sign of the rotation (CCW positive or negative?)
%    - Any ToF sensors that read max range or behave unexpectedly
%    - Whether the mag circle (Fig 5) looks calibrated or needs re-running
%
% =========================================================================
%
% Usage:
%   1. In Simulink, start a recording → perform the motion → stop
%   2. load('your_recording.mat')
%   3. Run this script — read the printed report and inspect all 8 figures.

clear; clc; close all;

% ---- Load data ---- (comment out the line that does not apply)
load('Training Data/task1_1 1.mat');       % offline training data
% (or load whatever file you bring to the lab)

if ~exist('out','var')
    error('Load a .mat file first: load(''Training Data/task1_1 1.mat'')');
end

%% =====================================================================
%  EXTRACT RAW SENSOR DATA
%  =====================================================================
imu_time   = out.Sensor_GYRO.time;
t          = imu_time - imu_time(1);              % relative time (s)
N          = length(t);

gyro_raw   = squeeze(out.Sensor_GYRO.signals.values)';   % Nx3  [rad/s]
accel_raw  = squeeze(out.Sensor_ACCEL.signals.values)';  % Nx3  [m/s^2]

mag_time   = out.Sensor_MAG.time;
mag_raw    = squeeze(out.Sensor_MAG.signals.values)';    % Nx3  [T]

tof1_data  = squeeze(out.Sensor_ToF1.signals.values);
tof2_data  = squeeze(out.Sensor_ToF2.signals.values);
tof3_data  = squeeze(out.Sensor_ToF3.signals.values);
if size(tof1_data,1)==4, tof1_data=tof1_data'; end
if size(tof2_data,1)==4, tof2_data=tof2_data'; end
if size(tof3_data,1)==4, tof3_data=tof3_data'; end

% GT (for reference if available)
gt_pos = squeeze(out.GT_position.signals.values);
if size(gt_pos,1)==3, gt_pos=gt_pos'; end

%% =====================================================================
%  1.  GYROSCOPE — identify yaw axis and bias
%  =====================================================================
fprintf('\n=======================================================\n');
fprintf('  GYROSCOPE ANALYSIS\n');
fprintf('=======================================================\n');

gyro_mean = mean(gyro_raw, 1);
gyro_std  = std(gyro_raw, 0, 1);
fprintf('Static mean (rad/s): axis1=%.5f  axis2=%.5f  axis3=%.5f\n', ...
        gyro_mean(1), gyro_mean(2), gyro_mean(3));
fprintf('Static std  (rad/s): axis1=%.5f  axis2=%.5f  axis3=%.5f\n', ...
        gyro_std(1),  gyro_std(2),  gyro_std(3));
fprintf('\nThe LARGEST |mean| axis is likely the gravity-aligned or most-biased axis.\n');
fprintf('For YAW (turning), look for the axis that changes most when the robot turns.\n');
fprintf('In this dataset: axis1 mean=%.5f rad/s → likely yaw (gyro_bias_x=%.5f)\n', ...
        gyro_mean(1), gyro_mean(1));

% Integrate each axis to get angular displacement
dt_imu = median(diff(t));
gyro_integrated = cumsum(gyro_raw .* dt_imu, 1);  % Nx3 cumulative angle [rad]

figure(1); clf;
subplot(3,1,1); plot(t, rad2deg(gyro_raw(:,1)),'b');   ylabel('Gyro Axis 1 (deg/s)'); title('Raw Gyroscope'); grid on;
subplot(3,1,2); plot(t, rad2deg(gyro_raw(:,2)),'r');   ylabel('Gyro Axis 2 (deg/s)'); grid on;
subplot(3,1,3); plot(t, rad2deg(gyro_raw(:,3)),'g');   ylabel('Gyro Axis 3 (deg/s)'); xlabel('Time (s)'); grid on;
sgtitle('Fig 1: Gyro raw  —  the yaw axis shows most change when robot turns');

figure(2); clf;
subplot(3,1,1); plot(t, rad2deg(gyro_integrated(:,1)),'b');  ylabel('Integrated axis1 (deg)'); title('Gyro integrated (cumulative angle)'); grid on;
subplot(3,1,2); plot(t, rad2deg(gyro_integrated(:,2)),'r');  ylabel('Integrated axis2 (deg)'); grid on;
subplot(3,1,3); plot(t, rad2deg(gyro_integrated(:,3)),'g');  ylabel('Integrated axis3 (deg)'); xlabel('Time (s)'); grid on;
sgtitle('Fig 2: Integrated gyro  —  yaw axis should track 360 deg for a full rotation');

%% =====================================================================
%  2.  ACCELEROMETER — identify gravity axis and body-frame layout
%  =====================================================================
fprintf('\n=======================================================\n');
fprintf('  ACCELEROMETER ANALYSIS\n');
fprintf('=======================================================\n');

accel_mean = mean(accel_raw, 1);
accel_std  = std(accel_raw, 0, 1);
fprintf('Static mean (m/s^2): axis1=%.4f  axis2=%.4f  axis3=%.4f\n', ...
        accel_mean(1), accel_mean(2), accel_mean(3));
fprintf('Static std  (m/s^2): axis1=%.4f  axis2=%.4f  axis3=%.4f\n', ...
        accel_std(1),  accel_std(2),  accel_std(3));

% Expected: one axis ≈ ±9.81 (gravity), the other two ≈ 0
[~, grav_axis] = max(abs(accel_mean));
fprintf('\n>>> Axis %d has the largest static reading (%.4f m/s^2) → gravity axis (VERTICAL).\n', ...
        grav_axis, accel_mean(grav_axis));
fprintf('    This axis does NOT measure robot horizontal acceleration.\n');
fprintf('    Remove gravity BEFORE using this axis for navigation:\n');
fprintf('    ax_corrected = accel_raw(:,%d) - %.5f\n', grav_axis, accel_mean(grav_axis));

% Tilt estimation from static accelerometer (valid only when stationary)
% roll = atan2(ay, az),  pitch = atan2(-ax, sqrt(ay^2+az^2))
roll_accel  = atan2d(accel_raw(:,2), accel_raw(:,3));
pitch_accel = atan2d(-accel_raw(:,1), sqrt(accel_raw(:,2).^2 + accel_raw(:,3).^2));

fprintf('\nStatic tilt (should be ~0 if robot is level):\n');
fprintf('  Mean roll  = %.2f deg\n', mean(roll_accel));
fprintf('  Mean pitch = %.2f deg\n', mean(pitch_accel));

figure(3); clf;
subplot(3,1,1); plot(t, accel_raw(:,1),'b'); hold on; yline(accel_mean(1),'k--','mean'); ylabel('Accel Axis 1 (m/s²)'); title('Raw Accelerometer'); grid on;
subplot(3,1,2); plot(t, accel_raw(:,2),'r'); hold on; yline(accel_mean(2),'k--','mean'); ylabel('Accel Axis 2 (m/s²)'); grid on;
subplot(3,1,3); plot(t, accel_raw(:,3),'g'); hold on; yline(accel_mean(3),'k--','mean'); ylabel('Accel Axis 3 (m/s²)'); xlabel('Time (s)'); grid on;
sgtitle('Fig 3: Accelerometer raw  —  one axis ≈ ±9.81 m/s² = gravity (vertical)');

figure(4); clf;
subplot(2,1,1); plot(t, roll_accel,  'b'); ylabel('Roll (deg)');  title('Tilt from accelerometer (valid when stationary)'); grid on;
subplot(2,1,2); plot(t, pitch_accel, 'r'); ylabel('Pitch (deg)'); xlabel('Time (s)'); grid on;
sgtitle('Fig 4: Roll/Pitch from accelerometer');

%% =====================================================================
%  3.  MAGNETOMETER — heading (yaw) and calibration check
%  =====================================================================
fprintf('\n=======================================================\n');
fprintf('  MAGNETOMETER ANALYSIS\n');
fprintf('=======================================================\n');

mag_mean = mean(mag_raw, 1);
mag_std  = std(mag_raw, 0, 1);
fprintf('Static mean (T): axis1=%.4e  axis2=%.4e  axis3=%.4e\n', ...
        mag_mean(1), mag_mean(2), mag_mean(3));
fprintf('Static std  (T): axis1=%.4e  axis2=%.4e  axis3=%.4e\n', ...
        mag_std(1),  mag_std(2),  mag_std(3));

% Calibrated heading using known calibration constants
mag_hard_iron = [-3.705e-05,  4.415e-05];   % [Y, Z] offsets
mag_soft_iron = [1.0958,      0.9196];       % [Y, Z] scale factors
mag_y_cal = (mag_raw(:,2) - mag_hard_iron(1)) .* mag_soft_iron(1);
mag_z_cal = (mag_raw(:,3) - mag_hard_iron(2)) .* mag_soft_iron(2);

heading_mag_raw = atan2d(mag_raw(:,3), mag_raw(:,2));
heading_mag_cal = atan2d(mag_z_cal,    mag_y_cal);

fprintf('\nMag heading range (raw calibrated):\n');
fprintf('  Raw: min=%.1f  max=%.1f  range=%.1f deg\n', ...
        min(heading_mag_raw), max(heading_mag_raw), range(heading_mag_raw));
fprintf('  Cal: min=%.1f  max=%.1f  range=%.1f deg\n', ...
        min(heading_mag_cal), max(heading_mag_cal), range(heading_mag_cal));
fprintf('(For a 360-deg rotation the range should be ~360 deg)\n');

figure(5); clf;
subplot(2,1,1);
plot(t, heading_mag_raw, 'r', 'DisplayName','raw'); hold on;
plot(t, heading_mag_cal, 'b', 'DisplayName','calibrated');
ylabel('Mag heading (deg)'); title('Magnetometer heading'); legend; grid on;
subplot(2,1,2);
plot(mag_raw(:,2), mag_raw(:,3), 'r.', 'DisplayName','raw'); hold on;
plot(mag_y_cal,    mag_z_cal,    'b.', 'DisplayName','calibrated');
xlabel('Mag Y'); ylabel('Mag Z'); axis equal;
title('Lissajous circle  —  calibrated should be a centred circle'); legend; grid on;
sgtitle('Fig 5: Magnetometer heading and calibration');

%% =====================================================================
%  4.  HEADING COMPARISON — gyro vs mag (yaw)
%  =====================================================================
fprintf('\n=======================================================\n');
fprintf('  HEADING COMPARISON (Gyro vs Mag)\n');
fprintf('=======================================================\n');

% Gyro yaw: integrate axis 1 with bias removed
gyro_bias_est = gyro_mean(1);
yaw_gyro_rad  = cumsum(-(gyro_raw(:,1) - gyro_bias_est) .* dt_imu);

% Mag yaw (calibrated, offset to start at same point)
mag_yaw_rad     = atan2(mag_z_cal, mag_y_cal);
mag_yaw_shifted = mag_yaw_rad - mag_yaw_rad(1) + yaw_gyro_rad(1);

fprintf('Gyro yaw drift over run: %.3f deg\n',  rad2deg(yaw_gyro_rad(end) - yaw_gyro_rad(1)));
fprintf('Mag  yaw change over run: %.3f deg\n', rad2deg(mag_yaw_shifted(end) - mag_yaw_shifted(1)));

figure(6); clf;
plot(t, rad2deg(yaw_gyro_rad),  'b-',  'LineWidth',1.5,'DisplayName','Gyro (integrated)'); hold on;
mag_t = out.Sensor_MAG.time - out.Sensor_MAG.time(1);
mag_yaw_interp = interp1(mag_t, mag_yaw_shifted, t, 'linear', 'extrap');
plot(t, rad2deg(mag_yaw_interp), 'r--', 'LineWidth',1.5,'DisplayName','Magnetometer');
ylabel('Yaw (deg)'); xlabel('Time (s)');
title('Heading estimate: gyro vs mag  —  should agree (gyro drifts long-term)');
legend; grid on;
sgtitle('Fig 6: Yaw comparison');

%% =====================================================================
%  5.  ToF SENSORS — distance readings and valid-flag analysis
%  =====================================================================
fprintf('\n=======================================================\n');
fprintf('  ToF SENSOR ANALYSIS\n');
fprintf('=======================================================\n');

sensors = {tof1_data, tof2_data, tof3_data};
names   = {'ToF1 (Forward, alpha=0)', 'ToF2 (Left, alpha=+90°)', 'ToF3 (Right, alpha=-90°)'};

for s = 1:3
    d    = sensors{s};
    dist = d(:,1);
    flag = d(:,4);    % 0 = valid
    valid = flag == 0;

    [~, ui] = unique(dist, 'stable');
    n_unique = length(ui);
    zoh_factor = length(dist) / n_unique;

    fprintf('\n%s:\n', names{s});
    fprintf('  Valid readings: %d / %d (%.1f%%)\n', sum(valid), length(valid), 100*mean(valid));
    fprintf('  Distance range: %.3f m to %.3f m\n', min(dist(valid)), max(dist(valid)));
    fprintf('  Mean distance (valid): %.3f m\n',    mean(dist(valid)));
    fprintf('  Unique values: %d → ZOH repeat factor: %.1fx (physical rate: %.1f Hz)\n', ...
            n_unique, zoh_factor, length(dist)/(zoh_factor*median(diff(t))*length(dist)));
end

tof_t1 = out.Sensor_ToF1.time - out.Sensor_ToF1.time(1);
tof_t2 = out.Sensor_ToF2.time - out.Sensor_ToF2.time(1);
tof_t3 = out.Sensor_ToF3.time - out.Sensor_ToF3.time(1);

figure(7); clf;
subplot(3,1,1);
valid1 = tof1_data(:,4)==0;
plot(tof_t1(valid1), tof1_data(valid1,1), 'b.','MarkerSize',3); ylabel('Dist (m)');
title('ToF1 — Forward (+X body)'); grid on;
subplot(3,1,2);
valid2 = tof2_data(:,4)==0;
plot(tof_t2(valid2), tof2_data(valid2,1), 'r.','MarkerSize',3); ylabel('Dist (m)');
title('ToF2 — Left (+Y body, faces y_{max} wall when \theta≈0)'); grid on;
subplot(3,1,3);
valid3 = tof3_data(:,4)==0;
plot(tof_t3(valid3), tof3_data(valid3,1), 'g.','MarkerSize',3); ylabel('Dist (m)');
xlabel('Time (s)');
title('ToF3 — Right (-Y body, faces y_{min} wall when \theta≈0)'); grid on;
sgtitle('Fig 7: ToF distance readings (valid only)');

%% =====================================================================
%  6.  SENSOR CONSISTENCY CHECK — ToF2 + ToF3 should sum to arena Y span
%  =====================================================================
fprintf('\n=======================================================\n');
fprintf('  SENSOR CONSISTENCY CHECK\n');
fprintf('=======================================================\n');

arena_y_span = 1.2 - (-2.16);  % 3.36 m
tof2_valid_dist = tof2_data(tof2_data(:,4)==0, 1);
tof3_valid_dist = tof3_data(tof3_data(:,4)==0, 1);

n_check = min(length(tof2_valid_dist), length(tof3_valid_dist));
tof_sum = tof2_valid_dist(1:n_check) + tof3_valid_dist(1:n_check);

fprintf('Arena Y span (y_max - y_min): %.3f m\n', arena_y_span);
fprintf('ToF2 + ToF3 mean: %.3f m  (should ≈ %.3f when theta≈0)\n', mean(tof_sum), arena_y_span);
fprintf('ToF2 + ToF3 std:  %.4f m  (small = sensors agree with arena model)\n', std(tof_sum));

figure(8); clf;
plot(1:n_check, tof_sum, 'b.', 'MarkerSize', 3); hold on;
yline(arena_y_span, 'r--', 'LineWidth', 2, 'DisplayName', 'Arena Y span (3.36 m)');
ylabel('ToF2 + ToF3 (m)'); xlabel('Sample');
title('Consistency: ToF2 + ToF3 ≈ arena Y span (3.36 m) when robot faces +X');
legend; grid on;
sgtitle('Fig 8: ToF2+ToF3 sum — deviation from 3.36 m = misalignment or wrong arena bounds');

%% =====================================================================
%  7.  PRINTED SUMMARY REPORT
%  =====================================================================
fprintf('\n=======================================================\n');
fprintf('  SUMMARY REPORT FOR LAB SESSION\n');
fprintf('=======================================================\n');
fprintf('\nGyroscope:\n');
fprintf('  Yaw axis: axis 1 (negated in EKF: omega = -(gyro(:,1) - %.5f))\n', gyro_mean(1));
fprintf('  Bias to remove: %.5f rad/s\n', gyro_mean(1));

fprintf('\nAccelerometer:\n');
fprintf('  Gravity axis: axis %d (mean = %.4f m/s^2 ≈ ±g)\n', grav_axis, accel_mean(grav_axis));
fprintf('  This axis is VERTICAL — subtract %.4f before using for navigation.\n', accel_mean(grav_axis));
fprintf('  WARNING: gravity leaks into other axes when robot TURNS.\n');
fprintf('           For Task 2 (circuit), DO NOT use raw accel for velocity — use ax=ay=0.\n');

fprintf('\nMagnetometer:\n');
fprintf('  Horizontal heading axes: Y (col 2) and Z (col 3)\n');
fprintf('  Hard-iron bias: [%.4e, %.4e] T\n', mag_hard_iron(1), mag_hard_iron(2));
fprintf('  Soft-iron scale: [%.4f, %.4f]\n', mag_soft_iron(1), mag_soft_iron(2));

fprintf('\nToF sensors:\n');
fprintf('  Physical update rate: ~10 Hz (logged at 200 Hz with 20x ZOH repeat)\n');
fprintf('  ToF1 (alpha=0):    forward, constrains X position\n');
fprintf('  ToF2 (alpha=+90°): left,    constrains Y position (toward y_max wall)\n');
fprintf('  ToF3 (alpha=-90°): right,   constrains Y position (toward y_min wall)\n');
fprintf('  ToF2 + ToF3 mean should equal %.2f m when theta ≈ 0.\n', arena_y_span);
fprintf('  If ToF2+ToF3 ≠ 3.36 m, either arena bounds are wrong or sensors are misaligned.\n');

fprintf('\nSensor roles in EKF:\n');
fprintf('  PREDICTION : gyro → theta,  accel → vx/vy (caution: gravity)\n');
fprintf('  MAG UPDATE : heading correction (drifts slowly, unreliable near metal)\n');
fprintf('  TOF UPDATE : absolute position from wall distances\n');
fprintf('\nDone. Check Figures 1-8.\n');
