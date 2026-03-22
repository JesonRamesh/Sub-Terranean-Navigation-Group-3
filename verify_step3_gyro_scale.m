%% verify_step3_gyro_scale.m — Session 0, Step 3: Gyro Scale Factor Verification
% Run this from the Coursework folder in MATLAB.
% Output is printed only — no files are saved, no variables are modified in myEKF.m.
%
% Maps to session doc:
%   3a — Integrate bias-corrected gyro over full rotation recording
%   3b — Print total angle and error vs 5-rotation ground truth
%   3c — Interpret error; compute scale factor correction if needed

clc;
fprintf('=============================================================\n');
fprintf(' Session 0 — Step 3: Gyro Scale Factor Verification\n');
fprintf('=============================================================\n\n');

% Verified values from Step 1
GYRO_BIAS_VERIFIED = 0.001855;   % rad/s (axis 1, from Step 1c)
YAW_AXIS           = 1;           % confirmed in Step 1b

% Ground truth: robot completes exactly 5 full rotations
N_ROTATIONS = 5;
EXPECTED_ANGLE = N_ROTATIONS * 2 * pi;   % = 31.4159 rad

fprintf('Loading Training Data/calib1_rotate.mat ...\n');
load('Training Data/calib1_rotate.mat');

gyro_data = squeeze(out.Sensor_GYRO.signals.values)';   % [N x 3]
time_data = out.Sensor_GYRO.time;                        % [N x 1]

N = length(time_data);
dt_vec = diff(time_data);
dt     = mean(dt_vec);
fs     = 1 / dt;

fprintf('Samples:              %d\n', N);
fprintf('Recording duration:   %.3f s\n', time_data(end) - time_data(1));
fprintf('Mean dt:              %.6f s\n', dt);
fprintf('Estimated rate:       %.2f Hz\n', fs);
fprintf('dt std dev:           %.2e s  (jitter check)\n', std(dt_vec));
fprintf('\n');

% ------------------------------------------------------------------ %
%  STEP 3a — Integrate bias-corrected gyro over full recording        %
% ------------------------------------------------------------------ %
fprintf('--- Step 3a: Integrate gyro ---\n');

gyro_yaw        = gyro_data(:, YAW_AXIS);                 % raw yaw-axis signal
gyro_corrected  = gyro_yaw - GYRO_BIAS_VERIFIED;          % bias-corrected

% Cumulative angle using trapezoidal integration (more accurate than cumsum×dt)
% trapz gives total; for cumulative we use cumtrapz
cumulative_angle = cumtrapz(time_data, gyro_corrected);   % [N x 1]

fprintf('Yaw axis:             %d\n', YAW_AXIS);
fprintf('Bias used:            %.6f rad/s\n', GYRO_BIAS_VERIFIED);
fprintf('Total integrated angle (end of recording): %.5f rad\n', cumulative_angle(end));
fprintf('Expected (5 rotations, CCW):               %+.5f rad\n', +EXPECTED_ANGLE);
fprintf('Expected (5 rotations, CW):                %+.5f rad\n', -EXPECTED_ANGLE);
fprintf('\n');

% ------------------------------------------------------------------ %
%  STEP 3b — Print error vs ground truth                              %
% ------------------------------------------------------------------ %
fprintf('--- Step 3b: Error analysis ---\n');

actual_angle = cumulative_angle(end);
% Error against both CW and CCW — use whichever is smaller
err_ccw = abs(actual_angle - EXPECTED_ANGLE);
err_cw  = abs(actual_angle + EXPECTED_ANGLE);

if err_ccw <= err_cw
    rotation_sign = +1;
    direction_str = 'CCW';
    ref_angle     = EXPECTED_ANGLE;
    angle_error   = actual_angle - EXPECTED_ANGLE;
else
    rotation_sign = -1;
    direction_str = 'CW';
    ref_angle     = -EXPECTED_ANGLE;
    angle_error   = actual_angle + EXPECTED_ANGLE;
end

pct_error = abs(angle_error) / EXPECTED_ANGLE * 100;

fprintf('Detected rotation direction: %s\n', direction_str);
fprintf('Reference angle:             %+.5f rad\n', ref_angle);
fprintf('Actual integrated angle:     %+.5f rad\n', actual_angle);
fprintf('Error:                       %+.5f rad  (%.3f%%)\n', angle_error, pct_error);
fprintf('\n');

% Per-revolution breakdown: sample every N/5 point to check linearity
fprintf('Per-revolution breakdown (cumulative angle at each 1/5 of recording):\n');
seg_len = floor(N / N_ROTATIONS);
for r = 1:N_ROTATIONS
    idx = r * seg_len;
    if idx > N, idx = N; end
    expected_r = rotation_sign * r * 2 * pi;
    actual_r   = cumulative_angle(idx);
    err_r      = actual_r - expected_r;
    fprintf('  After rotation %d: angle = %+.4f rad  (expected %+.4f,  err = %+.4f rad)\n', ...
        r, actual_r, expected_r, err_r);
end
fprintf('\n');

% ------------------------------------------------------------------ %
%  STEP 3c — Interpret and compute scale factor if needed             %
% ------------------------------------------------------------------ %
fprintf('--- Step 3c: Interpretation ---\n');

if pct_error < 2.0
    fprintf('Result: PASS — error < 2%%.  Gyro scale is fine.\n');
elseif pct_error < 5.0
    fprintf('Result: MINOR — error %.2f%% (2–5%%).  Acceptable but scale factor may help.\n', pct_error);
else
    fprintf('Result: SIGNIFICANT — error %.2f%% (> 5%%).  Scale factor correction is needed.\n', pct_error);
end

% Always print the scale factor so it's available regardless of outcome
scale_factor = ref_angle / actual_angle;
fprintf('\nGyro scale factor correction (ref / actual): %.6f\n', scale_factor);
fprintf('  (Apply as: omega = (gyro - bias) * %.6f)\n', scale_factor);
fprintf('  (Value of 1.000000 = no correction needed)\n');
fprintf('\n');

% Check whether a scale factor is currently applied in myEKF.m
% myEKF.m line 100: omega = gyro_data(k, 1) - gyro_bias_x;  (no scale factor)
fprintf('Scale factor in myEKF.m: NONE applied\n');
fprintf('  (myEKF.m uses: omega = gyro_data(k,1) - gyro_bias_x  — no multiplication)\n');
if abs(scale_factor - 1.0) > 0.02
    fprintf('*** DISCREPANCY *** scale factor %.6f differs from 1.0 by > 2%%.\n', scale_factor);
else
    fprintf('OK — scale factor close enough to 1.0 that omitting it is acceptable.\n');
end

% Additional diagnostic: min/max instantaneous rate
fprintf('\nInstantaneous gyro rate diagnostics (bias-corrected):\n');
fprintf('  Min:  %.5f rad/s\n', min(gyro_corrected));
fprintf('  Max:  %.5f rad/s\n', max(gyro_corrected));
fprintf('  Mean: %.5f rad/s  (should be near 0 if bias is correct)\n', mean(gyro_corrected));
fprintf('  Std:  %.5f rad/s\n', std(gyro_corrected));

fprintf('\n=============================================================\n');
fprintf(' Step 3 complete. Paste this output back to Claude.\n');
fprintf('=============================================================\n');
