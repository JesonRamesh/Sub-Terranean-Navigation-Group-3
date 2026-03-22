%% verify_step1_gyro.m — Session 0, Step 1: Gyro Bias Verification
% Run this from the Coursework folder in MATLAB.
% Output is printed only — no files are saved, no variables are modified in myEKF.m.
%
% Maps to session doc:
%   1a — Identify stationary window from calib2_straight.mat
%   1b — Identify yaw axis from calib1_rotate.mat
%   1c — Compute verified bias, sigma, R_gyro; compare to myEKF.m value
%   1d — Check stationary window sample count

clc;
fprintf('=============================================================\n');
fprintf(' Session 0 — Step 1: Gyro Bias Verification\n');
fprintf('=============================================================\n\n');

% ------------------------------------------------------------------ %
%  STEP 1a — Identify the stationary window (calib2_straight.mat)    %
% ------------------------------------------------------------------ %
fprintf('--- Step 1a: Stationary window identification ---\n');
fprintf('Loading Training Data/calib2_straight.mat ...\n');
load('Training Data/calib2_straight.mat');

% Data is stored as 3×N (axes × time); transpose to N×3 (time × axes)
gyro_straight  = squeeze(out.Sensor_GYRO.signals.values)';   % [N x 3]
time_straight  = out.Sensor_GYRO.time;                        % [N x 1]

% Gyro magnitude across all three axes
gyro_mag = sqrt(sum(gyro_straight.^2, 2));   % [N x 1]

fprintf('Total recording length: %.2f s  (%d samples)\n', ...
    time_straight(end), length(time_straight));

% Auto-detect stationary window: magnitude stays below threshold continuously
% from the very start.  Threshold = 0.05 rad/s (well below typical motion).
STAT_THRESH = 0.05;   % rad/s — adjust if detection looks wrong
stat_mask_straight = gyro_mag < STAT_THRESH;

% Find the latest contiguous block of True values starting at sample 1
stat_end_idx = 1;
while stat_end_idx < length(stat_mask_straight) && stat_mask_straight(stat_end_idx + 1)
    stat_end_idx = stat_end_idx + 1;
end

t_stat_start = time_straight(1);
t_stat_end   = time_straight(stat_end_idx);

fprintf('Stationary threshold used: %.3f rad/s\n', STAT_THRESH);
fprintf('Stationary window: %.3f s  to  %.3f s\n', t_stat_start, t_stat_end);
fprintf('Stationary window duration: %.3f s\n', t_stat_end - t_stat_start);

% Print gyro magnitude stats during this window
fprintf('Gyro magnitude in window — mean: %.5f rad/s,  max: %.5f rad/s\n', ...
    mean(gyro_mag(1:stat_end_idx)), max(gyro_mag(1:stat_end_idx)));

% Also print the boundary: first 3 samples just after the window
fprintf('Gyro magnitude at window boundary (samples %d-%d): ', ...
    stat_end_idx+1, min(stat_end_idx+3, length(gyro_mag)));
fprintf('%.4f  ', gyro_mag(stat_end_idx+1 : min(stat_end_idx+3, end)));
fprintf('\n\n');

% ------------------------------------------------------------------ %
%  STEP 1b — Identify the yaw axis (calib1_rotate.mat)               %
% ------------------------------------------------------------------ %
fprintf('--- Step 1b: Yaw axis identification from rotation data ---\n');
fprintf('Loading Training Data/calib1_rotate.mat ...\n');
load('Training Data/calib1_rotate.mat');

gyro_rotate = squeeze(out.Sensor_GYRO.signals.values)';   % [N x 3]
time_rotate = out.Sensor_GYRO.time;                        % [N x 1]

fprintf('Rotation recording length: %.2f s  (%d samples)\n', ...
    time_rotate(end), length(time_rotate));

% RMS of each axis across the entire rotation recording
rms_per_axis = sqrt(mean(gyro_rotate.^2, 1));   % [1 x 3]
[~, yaw_axis_detected] = max(rms_per_axis);

fprintf('\nRMS per gyro axis during rotation:\n');
for ax = 1:3
    marker = '';
    if ax == yaw_axis_detected, marker = '  <-- LARGEST (detected yaw axis)'; end
    fprintf('  Axis %d: RMS = %.5f rad/s%s\n', ax, rms_per_axis(ax), marker);
end

% Axis used in myEKF.m
MYEKF_GYRO_AXIS = 1;
fprintf('\nAxis used in myEKF.m: axis %d\n', MYEKF_GYRO_AXIS);
if yaw_axis_detected == MYEKF_GYRO_AXIS
    fprintf('MATCH — detected yaw axis agrees with myEKF.m.\n');
else
    fprintf('*** MISMATCH *** — detected yaw axis %d but myEKF.m uses axis %d.\n', ...
        yaw_axis_detected, MYEKF_GYRO_AXIS);
end

% Also show peak (not just RMS) for confirmation
peak_per_axis = max(abs(gyro_rotate), [], 1);
fprintf('\nPeak |gyro| per axis during rotation:\n');
for ax = 1:3
    fprintf('  Axis %d: peak = %.5f rad/s\n', ax, peak_per_axis(ax));
end
fprintf('\n');

% ------------------------------------------------------------------ %
%  STEP 1c — Compute verified bias, sigma, R_gyro                    %
% ------------------------------------------------------------------ %
fprintf('--- Step 1c: Compute verified gyro bias ---\n');

% Reload straight file (it was overwritten by calib1_rotate load above)
load('Training Data/calib2_straight.mat');
gyro_straight = squeeze(out.Sensor_GYRO.signals.values)';
time_straight = out.Sensor_GYRO.time;

% Rebuild stationary mask with the same threshold
gyro_mag2 = sqrt(sum(gyro_straight.^2, 2));
stat_mask2 = gyro_mag2 < STAT_THRESH;
stat_end_idx2 = 1;
while stat_end_idx2 < length(stat_mask2) && stat_mask2(stat_end_idx2 + 1)
    stat_end_idx2 = stat_end_idx2 + 1;
end
stat_samples = gyro_straight(1:stat_end_idx2, :);

gyro_bias_verified = mean(stat_samples(:, yaw_axis_detected));
gyro_noise_sigma   = std(stat_samples(:, yaw_axis_detected));
R_gyro_verified    = gyro_noise_sigma^2;

fprintf('Yaw axis (detected):   %d\n', yaw_axis_detected);
fprintf('gyro_bias_verified:    %.6f rad/s\n', gyro_bias_verified);
fprintf('gyro_noise_sigma:      %.6f rad/s\n', gyro_noise_sigma);
fprintf('R_gyro_verified:       %.6e (rad/s)^2\n', R_gyro_verified);

% Compare to myEKF.m
MYEKF_GYRO_BIAS = 0.00186;   % rad/s — current value in myEKF.m line 34
bias_diff = abs(gyro_bias_verified - MYEKF_GYRO_BIAS);
fprintf('\nmyEKF.m gyro_bias_x:   %.5f rad/s\n', MYEKF_GYRO_BIAS);
fprintf('Difference:            %.6f rad/s\n', bias_diff);
if bias_diff > 0.001
    fprintf('*** DISCREPANCY *** — difference > 0.001 rad/s threshold.\n');
else
    fprintf('OK — difference within 0.001 rad/s threshold.\n');
end
fprintf('\n');

% ------------------------------------------------------------------ %
%  STEP 1d — Stationary window sample count                          %
% ------------------------------------------------------------------ %
fprintf('--- Step 1d: Stationary window sample count ---\n');
n_stat = stat_end_idx2;
dt_est = mean(diff(time_straight));
fs_est = 1 / dt_est;

fprintf('Samples in stationary window: %d\n', n_stat);
fprintf('Estimated sample rate:        %.1f Hz\n', fs_est);
fprintf('Equivalent duration:          %.2f s\n', n_stat * dt_est);
fprintf('Recommended minimum:          520 samples (5 s at 104 Hz)\n');

if n_stat < 520
    fprintf('*** CONCERN *** — fewer than 520 samples in stationary window.\n');
    fprintf('   Bias estimate may be less reliable.\n');
else
    fprintf('OK — sample count meets the 520-sample minimum.\n');
end

fprintf('\n=============================================================\n');
fprintf(' Step 1 complete. Paste this output back to Claude.\n');
fprintf('=============================================================\n');
