%% verify_step2_accel.m — Session 0, Step 2: Accelerometer Bias and Axis Mapping
% Run this from the Coursework folder in MATLAB.
% Output is printed only — no files are saved, no variables are modified in myEKF.m.
%
% Maps to session doc:
%   2a — Mean and std for all three accel axes during stationary window
%   2b — Identify forward axis from moving phase of calib2_straight.mat
%   2c — Identify lateral axis (remaining horizontal axis)
%   2d — Verify axis mapping and biases against myEKF.m
%   2e — Compute R_accel_verified for the forward axis

clc;
fprintf('=============================================================\n');
fprintf(' Session 0 — Step 2: Accelerometer Bias and Axis Mapping\n');
fprintf('=============================================================\n\n');

% Stationary threshold (same as Step 1 — gyro magnitude below this = stationary)
STAT_THRESH = 0.05;  % rad/s

fprintf('Loading Training Data/calib2_straight.mat ...\n');
load('Training Data/calib2_straight.mat');

% Gyro (to rebuild stationary mask)
gyro   = squeeze(out.Sensor_GYRO.signals.values)';   % [N x 3]
time_g = out.Sensor_GYRO.time;                        % [N x 1]

% Accel
accel   = squeeze(out.Sensor_ACCEL.signals.values)';  % [N x 3]
time_a  = out.Sensor_ACCEL.time;                       % [N x 1]

% --- Rebuild stationary mask using gyro magnitude (same method as Step 1) ---
gyro_mag = sqrt(sum(gyro.^2, 2));
stat_mask_g = gyro_mag < STAT_THRESH;
stat_end_idx = 1;
while stat_end_idx < length(stat_mask_g) && stat_mask_g(stat_end_idx + 1)
    stat_end_idx = stat_end_idx + 1;
end
t_stat_end = time_g(stat_end_idx);

% Map stationary mask to accel time axis
stat_mask_a = time_a <= t_stat_end;
% Moving mask: everything after stationary window
move_mask_a = time_a > t_stat_end;

fprintf('Stationary window end (from gyro mask): %.3f s\n', t_stat_end);
fprintf('Stationary samples (accel): %d\n', sum(stat_mask_a));
fprintf('Moving samples    (accel): %d\n\n', sum(move_mask_a));

% ------------------------------------------------------------------ %
%  STEP 2a — Mean and std for all three axes during stationary window %
% ------------------------------------------------------------------ %
fprintf('--- Step 2a: Stationary accel stats (all three axes) ---\n');

accel_stat = accel(stat_mask_a, :);

for i = 1:3
    m = mean(accel_stat(:, i));
    s = std(accel_stat(:, i));
    gravity_flag = '';
    if abs(m) > 8.0
        gravity_flag = '  <-- GRAVITY AXIS (|mean| > 8 m/s^2)';
    end
    fprintf('Accel axis %d:  mean = %+9.5f m/s^2,  std = %.5f m/s^2%s\n', ...
        i, m, s, gravity_flag);
end
fprintf('\n');

% Identify vertical axis (|mean| closest to 9.81)
[~, vertical_axis] = max(abs(mean(accel_stat, 1)));
fprintf('Identified VERTICAL axis (gravity): column %d\n', vertical_axis);
fprintf('  (This axis must NOT be used for velocity integration)\n\n');

% ------------------------------------------------------------------ %
%  STEP 2b — Identify forward axis from moving phase                  %
% ------------------------------------------------------------------ %
fprintf('--- Step 2b: Forward axis identification from moving phase ---\n');

accel_move = accel(move_mask_a, :);
time_move  = time_a(move_mask_a);

% Std deviation of each axis during motion (after subtracting stationary mean)
accel_stat_mean = mean(accel_stat, 1);   % [1 x 3] — stationary biases

% Bias-corrected motion signal
accel_move_corrected = accel_move - accel_stat_mean;

% Range (peak-to-peak) and std during motion for each axis
move_std   = std(accel_move_corrected, [], 1);
move_range = max(accel_move_corrected, [], 1) - min(accel_move_corrected, [], 1);

fprintf('During moving phase (bias-corrected from stationary mean):\n');
for i = 1:3
    skip = '';
    if i == vertical_axis, skip = '  (skip — vertical axis)'; end
    fprintf('  Axis %d:  std = %.5f m/s^2,  peak-to-peak = %.5f m/s^2%s\n', ...
        i, move_std(i), move_range(i), skip);
end

% Among horizontal axes only, find the one with the largest std during motion
horiz_axes = setdiff(1:3, vertical_axis);
[~, rel_idx] = max(move_std(horiz_axes));
forward_axis = horiz_axes(rel_idx);
lateral_axis = horiz_axes(setdiff(1:length(horiz_axes), rel_idx));

fprintf('\nIdentified FORWARD axis (largest motion std, horizontal only): column %d\n', forward_axis);

% ------------------------------------------------------------------ %
%  STEP 2c — Lateral axis                                             %
% ------------------------------------------------------------------ %
fprintf('\n--- Step 2c: Lateral axis ---\n');
fprintf('Identified LATERAL axis (remaining horizontal): column %d\n\n', lateral_axis);

% ------------------------------------------------------------------ %
%  STEP 2d — Verify axis mapping and biases in myEKF.m               %
% ------------------------------------------------------------------ %
fprintf('--- Step 2d: Verify myEKF.m axis mapping and biases ---\n');

% Current myEKF.m values (lines 34-36)
MYEKF_ACCEL_BIAS_AXIS1 = 9.84855;   % bias for accel axis 1 in myEKF.m
MYEKF_ACCEL_BIAS_AXIS2 = 0.07485;   % bias for accel axis 2 in myEKF.m
% myEKF.m uses: ax = accel_data(k,1) - accel_bias_x  -> integrates into vx (body forward)
%               ay = accel_data(k,2) - accel_bias_y  -> integrates into vy (body lateral)
MYEKF_FORWARD_AXIS = 1;   % axis used for forward (vx) integration in myEKF.m
MYEKF_LATERAL_AXIS = 2;   % axis used for lateral (vy) integration in myEKF.m

fprintf('myEKF.m forward (vx) axis: column %d  (bias = %.5f m/s^2)\n', ...
    MYEKF_FORWARD_AXIS, MYEKF_ACCEL_BIAS_AXIS1);
fprintf('myEKF.m lateral (vy) axis: column %d  (bias = %.5f m/s^2)\n', ...
    MYEKF_LATERAL_AXIS, MYEKF_ACCEL_BIAS_AXIS2);
fprintf('\n');

% Check: axis 1 bias in myEKF.m is 9.84855 — is that the vertical or forward axis?
fprintf('Verified axis assignments:\n');
fprintf('  Vertical axis  = column %d  (|mean| = %.5f m/s^2)\n', ...
    vertical_axis, abs(mean(accel_stat(:, vertical_axis))));
fprintf('  Forward  axis  = column %d  (stationary mean = %+.5f m/s^2)\n', ...
    forward_axis, mean(accel_stat(:, forward_axis)));
fprintf('  Lateral  axis  = column %d  (stationary mean = %+.5f m/s^2)\n', ...
    lateral_axis, mean(accel_stat(:, lateral_axis)));
fprintf('\n');

% Critical check: is the axis myEKF.m calls "forward" actually forward?
if MYEKF_FORWARD_AXIS == vertical_axis
    fprintf('*** CRITICAL BUG *** myEKF.m integrates the VERTICAL axis into vx.\n');
    fprintf('   accel axis %d has mean %.5f m/s^2 (gravity), not forward acceleration.\n', ...
        vertical_axis, mean(accel_stat(:, vertical_axis)));
    fprintf('   After bias subtraction: residual = %.5f m/s^2 (not zero — bias was from wrong data).\n', ...
        mean(accel_stat(:, vertical_axis)) - MYEKF_ACCEL_BIAS_AXIS1);
elseif MYEKF_FORWARD_AXIS == forward_axis
    fprintf('OK — myEKF.m forward axis matches verified forward axis.\n');
else
    fprintf('*** MISMATCH *** myEKF.m forward axis (%d) != verified forward axis (%d).\n', ...
        MYEKF_FORWARD_AXIS, forward_axis);
end

if MYEKF_LATERAL_AXIS == lateral_axis
    fprintf('OK — myEKF.m lateral axis matches verified lateral axis.\n');
elseif MYEKF_LATERAL_AXIS == vertical_axis
    fprintf('*** CRITICAL BUG *** myEKF.m integrates the VERTICAL axis into vy.\n');
else
    fprintf('*** MISMATCH *** myEKF.m lateral axis (%d) != verified lateral axis (%d).\n', ...
        MYEKF_LATERAL_AXIS, lateral_axis);
end

% Bias residual check for the forward axis: what does myEKF.m actually remove?
fprintf('\nBias residual analysis:\n');
bias_in_code = [MYEKF_ACCEL_BIAS_AXIS1, MYEKF_ACCEL_BIAS_AXIS2, 0];
for i = 1:2
    ax_code = i;  % myEKF.m processes axes 1 and 2
    verified_mean = mean(accel_stat(:, ax_code));
    residual = verified_mean - bias_in_code(ax_code);
    fprintf('  Axis %d: verified mean = %+.5f,  myEKF.m bias = %+.5f,  residual = %+.5f m/s^2\n', ...
        ax_code, verified_mean, bias_in_code(ax_code), residual);
end
fprintf('\n');

% ------------------------------------------------------------------ %
%  STEP 2e — Verified horizontal noise R_accel                        %
% ------------------------------------------------------------------ %
fprintf('--- Step 2e: Verified horizontal noise R_accel ---\n');

sigma_fwd = std(accel_stat(:, forward_axis));
R_accel_verified = sigma_fwd^2;

fprintf('Forward axis (col %d) stationary std:  %.6f m/s^2\n', forward_axis, sigma_fwd);
fprintf('R_accel_verified:                      %.6e (m/s^2)^2\n', R_accel_verified);

% myEKF.m has no explicit R_accel — the accel is used directly in the
% prediction step (not in a measurement update), so its noise enters via Q.
% Q_vx = 0.1 in myEKF.m — print comparison
MYEKF_Q_VX = 0.1;
fprintf('\nFor comparison:\n');
fprintf('  Q_vx in myEKF.m:     %.4f (m/s^2)^2 per step\n', MYEKF_Q_VX);
fprintf('  R_accel_verified:    %.2e (m/s^2)^2\n', R_accel_verified);
fprintf('  Ratio Q_vx / R_accel_verified: %.1f x\n', MYEKF_Q_VX / R_accel_verified);
if MYEKF_Q_VX / R_accel_verified > 100
    fprintf('  *** Q_vx is >> measured accel noise — see Step 5 for full analysis.\n');
end

fprintf('\n=============================================================\n');
fprintf(' Step 2 complete. Paste this output back to Claude.\n');
fprintf('=============================================================\n');
