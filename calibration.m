%% calibrate_sensors.m
% Derives all EKF calibration constants from the two calibration datasets.
%
% BOARD ORIENTATION: STM32 IKS02A1 rotated -90 degrees about its X axis.
% This maps sensor axes to robot/world frame as:
%   Board X --> World X  (forward)         [unchanged]
%   Board Y --> World -Z (vertically down)
%   Board Z --> World +Y (leftward)
%
% Sensor axis consequences:
%   Yaw rate        = -gyro(2, :)          Board Y = world -Z, so negate
%   Mag horizontal  = mag(1,:) and mag(3,:) Board X (fwd) and Board Z (left)
%   Mag vertical    =  mag(2, :)           Board Y -- NOT used for heading
%
% USAGE:
%   1. Run this script once before running myEKF.
%   2. Copy the printed constants into the calibration section of myEKF.m.
%
% OUTPUTS (printed to console, ready to paste into myEKF.m):
%   gyro_bias   [1x3] rad/s
%   gyro_scale  scalar
%   mag_hard    [1x2]  [offset_X, offset_Z] Tesla
%   mag_soft    [1x2]  [scale_X,  scale_Z]  dimensionless
%   R_tof       scalar (average of three ToF variance measurements)
%   gyro_noise_var, accel_noise_var for Q matrix tuning

clear; clc;

%% =========================================================================
%  PART 1 — STRAIGHT-LINE FILE (gyro bias, accel bias, ToF noise)
%  Source: calib2_straight.mat
%  Robot is stationary for ~60s, then drives straight forward and back.
% ==========================================================================
load('Training Data/calib2_straight.mat');

% --- Stationary window ---
% From the ToF plot the robot starts moving at ~2.5s, so cap at 2.0s.
% The first 0.2s is trimmed on the gyro to exclude placement transient spikes.
static_time_limit = 2.0;   % seconds — robot visibly moves after this
gyro_trim_start   = 0.2;   % seconds — skip startup vibration transients

% -------------------------------------------------------------------------
%  Gyroscope bias and noise
%  After -90 deg X rotation, yaw axis is Board Y (index 2).
%  All three axes are stored for completeness; only Y is used for yaw.
% -------------------------------------------------------------------------
gyro_raw  = squeeze(out.Sensor_GYRO.signals.values);  % [3 x N]
time_gyro = out.Sensor_GYRO.time;                     % [N x 1]

% Trim first 0.2s (placement transients) AND cap at static_time_limit
stat_mask_g = time_gyro >= (time_gyro(1) + gyro_trim_start) & ...
              time_gyro <  (time_gyro(1) + static_time_limit);

% Bias = mean reading per axis during the stationary window
gyro_bias     = mean(gyro_raw(:, stat_mask_g), 2);    % [3 x 1]

% Noise variance per axis during the stationary window (after bias removal)
gyro_corrected   = gyro_raw - gyro_bias;
gyro_noise_var   = var(gyro_corrected(:, stat_mask_g), 0, 2);  % [3 x 1]

% -------------------------------------------------------------------------
%  Accelerometer bias and noise (stored for reference / Q tuning)
%  Not directly used in the EKF position model but useful for future work.
% -------------------------------------------------------------------------
accel_raw  = squeeze(out.Sensor_ACCEL.signals.values);  % [3 x N]
time_accel = out.Sensor_ACCEL.time;

stat_mask_a  = time_accel < (time_accel(1) + static_time_limit);
accel_bias   = mean(accel_raw(:, stat_mask_a), 2);      % [3 x 1]
accel_noise_var = var(accel_raw(:, stat_mask_a), 0, 2); % [3 x 1]

% -------------------------------------------------------------------------
%  ToF measurement noise (stationary variance = sensor noise floor)
%  Each sensor may have a different sample count so each gets its own mask.
% -------------------------------------------------------------------------
sensors = {'Sensor_ToF1', 'Sensor_ToF2', 'Sensor_ToF3'};
tof_var  = nan(1, 3);
tof_mean = nan(1, 3);

for i = 1:3
    raw = squeeze(out.(sensors{i}).signals.values);  % [4 x N] or [N x 4]
    if size(raw, 1) == 4, raw = raw'; end            % ensure [N x 4]

    t_tof    = out.(sensors{i}).time;
    dist     = raw(:, 1);
    status   = raw(:, 4);

    stat_mask_t = t_tof < (t_tof(1) + static_time_limit);
    valid       = status == 0;                       % status 0 = good reading
    good        = stat_mask_t & valid;

    if sum(good) < 3
        warning('ToF%d: fewer than 3 valid stationary samples — widen static_time_limit', i);
        continue
    end

    tof_mean(i) = mean(dist(good));
    tof_var(i)  = var(dist(good));
end

% R_tof for the EKF: use the mean variance across the three sensors.
% You can also use individual values if one sensor is notably noisier.
R_tof_computed = mean(tof_var, 'omitnan');

%% =========================================================================
%  PART 2 — ROTATE FILE (magnetometer calibration + gyro scale)
%  Source: calib1_rotate.mat
%  Robot rotates continuously for exactly 5 complete cycles (5 * 2*pi rad).
%
%  ORDER: mag axis detection runs FIRST so horiz_axes is known before
%  the gyro scale section needs it to identify the vertical (yaw) axis.
% ==========================================================================
load('Training Data/calib1_rotate.mat');

% -------------------------------------------------------------------------
%  STEP 2a — Magnetometer axis auto-detection
%
%  During a full 360 deg horizontal rotation each horizontal axis sweeps a
%  complete sinusoid; the vertical axis barely moves.
%  Score all three axis pairs and pick the one where both axes have large,
%  roughly equal ranges — that is the horizontal plane.
% -------------------------------------------------------------------------
mag_rot   = squeeze(out.Sensor_MAG.signals.values);   % [3 x N]
range_all = max(mag_rot, [], 2) - min(mag_rot, [], 2); % [3 x 1] peak-to-peak

pairs      = [1 2; 1 3; 2 3];
pair_score = zeros(3, 1);
for p = 1:3
    a = pairs(p, 1);  b = pairs(p, 2);
    ra = range_all(a);  rb = range_all(b);
    % Geometric mean of ranges (both large) x circularity (equal ranges)
    pair_score(p) = sqrt(ra * rb) * min(ra, rb) / max(ra, rb);
end

[~, best_pair_idx] = max(pair_score);
horiz_axes = pairs(best_pair_idx, :);   % e.g. [1 3] or [2 3]
ax_a = horiz_axes(1);
ax_b = horiz_axes(2);

fprintf('\n--- Mag axis auto-detection ---\n');
fprintf('Axis ranges:  X=%.3e  Y=%.3e  Z=%.3e T\n', range_all);
fprintf('Detected horizontal pair: axes %d and %d  (scores: %.3e  %.3e  %.3e)\n', ...
        ax_a, ax_b, pair_score(1), pair_score(2), pair_score(3));

% -------------------------------------------------------------------------
%  STEP 2b — Magnetometer hard-iron and soft-iron correction
% -------------------------------------------------------------------------

% --- Hard-iron offsets for the detected horizontal pair ---
hard_iron_a = (max(mag_rot(ax_a, :)) + min(mag_rot(ax_a, :))) / 2;
hard_iron_b = (max(mag_rot(ax_b, :)) + min(mag_rot(ax_b, :))) / 2;

% --- Apply hard-iron correction ---
mag_corr = mag_rot;
mag_corr(ax_a, :) = mag_rot(ax_a, :) - hard_iron_a;
mag_corr(ax_b, :) = mag_rot(ax_b, :) - hard_iron_b;

% --- Soft-iron scale factors ---
amp_a   = (max(mag_corr(ax_a, :)) - min(mag_corr(ax_a, :))) / 2;
amp_b   = (max(mag_corr(ax_b, :)) - min(mag_corr(ax_b, :))) / 2;
avg_amp = (amp_a + amp_b) / 2;

soft_iron_a = avg_amp / amp_a;
soft_iron_b = avg_amp / amp_b;

% --- Apply soft-iron correction for sanity check ---
mag_final_a = mag_corr(ax_a, :) * soft_iron_a;
mag_final_b = mag_corr(ax_b, :) * soft_iron_b;
radius      = sqrt(mag_final_a.^2 + mag_final_b.^2);
ellipticity = std(radius) / mean(radius);   % 0 = perfect circle

% Map detected axes back to named variables for summary printout
hard_iron_x = hard_iron_a;   % offset for ax_a
hard_iron_z = hard_iron_b;   % offset for ax_b
soft_iron_x = soft_iron_a;   % scale  for ax_a
soft_iron_z = soft_iron_b;   % scale  for ax_b
mag_final_x = mag_final_a;
mag_final_z = mag_final_b;

% -------------------------------------------------------------------------
%  STEP 2c — Gyroscope scale factor
%
%  Integrate the yaw-axis gyro (Board Y = index 2, bias-corrected) over the
%  entire rotate dataset. True total angle = 5 complete rotations = 5*2*pi.
%  abs() handles either CW or CCW rotation direction.
%
%  gyro_bias is carried over from Part 1 (calib2_straight stationary window).
%  This is valid because bias is a sensor property, not dataset-dependent.
% -------------------------------------------------------------------------
gyro_rot      = squeeze(out.Sensor_GYRO.signals.values);  % [3 x N]
time_grot     = out.Sensor_GYRO.time;                     % [N x 1]

gyro_rot_corr = gyro_rot - gyro_bias;                     % remove bias on all axes
raw_angle     = trapz(time_grot, abs(gyro_rot_corr(1, :)));  % integrate |yaw rate| on Board X (index 1)
true_angle    = 5 * 2 * pi;                               % known: 5 full rotations
gyro_scale    = true_angle / raw_angle;                   % dimensionless scale factor

fprintf('Gyro scale raw integral: %.4f rad  (true = %.4f rad)\n', raw_angle, true_angle);

%% =========================================================================
%  SUMMARY — print constants ready to paste into myEKF.m
% ==========================================================================
fprintf('\n========================================================\n');
fprintf('  CALIBRATION SUMMARY  (paste into myEKF.m)\n');
fprintf('  Board orientation: Board X = vertical (world +Z)\n');
fprintf('  Yaw axis: Board X (index 1), no negation needed\n');
fprintf('  Mag horizontal axes: Board Y (2) and Board Z (3)\n');
fprintf('========================================================\n\n');

fprintf('--- Gyroscope ---\n');
fprintf('gyro_bias  = [%.6f, %.6f, %.6f];  %% [X, Y, Z] rad/s  (X is yaw axis)\n', ...
        gyro_bias(1), gyro_bias(2), gyro_bias(3));
fprintf('gyro_scale = %.4f;                             %% dimensionless\n', gyro_scale);
fprintf('gyro_noise_var = [%.2e, %.2e, %.2e];  %% for Q tuning\n\n', ...
        gyro_noise_var(1), gyro_noise_var(2), gyro_noise_var(3));

fprintf('--- Accelerometer (for reference / Q tuning) ---\n');
fprintf('accel_bias = [%.5f, %.5f, %.5f];  %% [X, Y, Z] m/s^2\n', ...
        accel_bias(1), accel_bias(2), accel_bias(3));
fprintf('accel_noise_var = [%.2e, %.2e, %.2e];\n\n', ...
        accel_noise_var(1), accel_noise_var(2), accel_noise_var(3));

fprintf('--- Magnetometer ---\n');
fprintf('Horizontal axes detected: %d (forward) and %d (left)\n', ax_a, ax_b);
fprintf('mag_hard = [%.4e, %.4e];  %% [offset_axis%d, offset_axis%d] Tesla\n', ...
        hard_iron_x, hard_iron_z, ax_a, ax_b);
fprintf('mag_soft = [%.4f, %.4f];             %% [scale_axis%d, scale_axis%d]\n', ...
        soft_iron_x, soft_iron_z, ax_a, ax_b);
fprintf('(Calibration ellipticity residual: %.4f — target < 0.05)\n', ellipticity);
fprintf('\n  >> In myEKF.m magnetometer update, use:\n');
fprintf('     mx = (mag_data(mag_idx, %d) - mag_hard(1)) * mag_soft(1);\n', ax_a);
fprintf('     mz = (mag_data(mag_idx, %d) - mag_hard(2)) * mag_soft(2);\n', ax_b);
fprintf('     z_mag = wrapToPi(atan2(mz, mx));\n\n');

fprintf('--- ToF sensors ---\n');
fprintf('tof_mean = [%.4f, %.4f, %.4f] m  (stationary distance to wall)\n', tof_mean);
fprintf('tof_std  = [%.5f, %.5f, %.5f] m\n', sqrt(tof_var));
fprintf('R_tof    = %.6f;  %% mean variance across sensors (m^2)\n\n', R_tof_computed);

fprintf('========================================================\n');
fprintf('  ACTION: copy the values above into myEKF.m sections 2/3\n');
fprintf('========================================================\n');

%% =========================================================================
%  DIAGNOSTIC PLOTS
% ==========================================================================

figure('Name', 'Gyro — stationary bias check', 'NumberTitle', 'off');
axis_labels = {'X', 'Y (yaw)', 'Z'};
for ax = 1:3
    subplot(3, 1, ax);
    t_plot = time_gyro(stat_mask_g);
    plot(t_plot, gyro_raw(ax, stat_mask_g), 'b'); hold on;
    yline(gyro_bias(ax), 'r--', sprintf('bias = %.5f', gyro_bias(ax)));
    ylabel(sprintf('Gyro %s (rad/s)', axis_labels{ax}));
    grid on;
    if ax == 1
        title(sprintf('Gyro stationary window (%.1fs–%.1fs) — verify flat signal', ...
              gyro_trim_start, static_time_limit));
    end
end
xlabel('Time (s)');

figure('Name', 'Mag axis pair check — identify horizontal axes', 'NumberTitle', 'off');
subplot(1, 3, 1);
plot(mag_rot(1, :), mag_rot(2, :), 'b.', 'MarkerSize', 2);
title('X vs Y'); axis equal; grid on;
xlabel('Mag X (T)'); ylabel('Mag Y (T)');

subplot(1, 3, 2);
plot(mag_rot(1, :), mag_rot(3, :), 'b.', 'MarkerSize', 2);
title('X vs Z'); axis equal; grid on;
xlabel('Mag X (T)'); ylabel('Mag Z (T)');

subplot(1, 3, 3);
plot(mag_rot(2, :), mag_rot(3, :), 'b.', 'MarkerSize', 2);
title('Y vs Z'); axis equal; grid on;
xlabel('Mag Y (T)'); ylabel('Mag Z (T)');
sgtitle('Pair that forms a full ellipse = the two horizontal axes');

figure('Name', 'Mag X-Z scatter — calibration check', 'NumberTitle', 'off');
subplot(1, 2, 1);
plot(mag_rot(ax_a, :), mag_rot(ax_b, :), 'b.', 'MarkerSize', 2);
xlabel(sprintf('Mag axis %d (T)', ax_a)); ylabel(sprintf('Mag axis %d (T)', ax_b));
title('Raw (should be ellipse off-centre)');
axis equal; grid on;

subplot(1, 2, 2);
plot(mag_final_x, mag_final_z, 'g.', 'MarkerSize', 2);
xlabel(sprintf('Axis %d corrected', ax_a)); ylabel(sprintf('Axis %d corrected', ax_b));
title(sprintf('Corrected (should be circle at origin)\nEllipticity = %.4f', ellipticity));
axis equal; grid on;
sgtitle(sprintf('Horizontal axes: %d and %d', ax_a, ax_b));

figure('Name', 'ToF stationary distributions', 'NumberTitle', 'off');
for i = 1:3
    raw = squeeze(out.(sensors{i}).signals.values);
    if size(raw, 1) == 4, raw = raw'; end
    t_tof  = out.(sensors{i}).time;
    dist   = raw(:, 1);
    status = raw(:, 4);
    stat_mask_t = t_tof < (t_tof(1) + static_time_limit);
    good = stat_mask_t & (status == 0);

    subplot(3, 1, i);
    plot(t_tof(good), dist(good), 'b.');
    yline(tof_mean(i), 'r--', sprintf('mean = %.4f m', tof_mean(i)));
    ylabel(sprintf('ToF%d dist (m)', i)); grid on;
    if i == 1, title('ToF stationary window — verify flat signal'); end
end
xlabel('Time (s)');