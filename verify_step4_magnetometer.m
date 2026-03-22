%% verify_step4_magnetometer.m — Session 0, Step 4: Magnetometer Calibration Verification
% Run this from the Coursework folder in MATLAB.
% Output is printed only — no files are saved, no variables are modified in myEKF.m.
%
% Maps to session doc:
%   4a — Characterise raw magnetometer Lissajous shape (Y vs Z, the horizontal axes)
%   4b — Compute hard-iron offsets; compare to myEKF.m
%   4c — Compute soft-iron scale factors; compare to myEKF.m
%   4d — Measure magnetometer noise sigma from stationary window; compare to R_mag = 100.0

clc;
fprintf('=============================================================\n');
fprintf(' Session 0 — Step 4: Magnetometer Calibration Verification\n');
fprintf('=============================================================\n\n');

% From myEKF.m (lines 39-40):
%   Horizontal axes: Y = column 2, Z = column 3
%   mag_hard_iron = [-3.705e-05,  4.415e-05]   [Y, Z]
%   mag_soft_iron = [1.0958,      0.9196]       [Y, Z]
%   Heading: atan2(mag_z_cal, mag_y_cal)
MYEKF_HARD_IRON_Y = -3.705e-05;
MYEKF_HARD_IRON_Z =  4.415e-05;
MYEKF_SOFT_IRON_Y =  1.0958;
MYEKF_SOFT_IRON_Z =  0.9196;
MYEKF_R_MAG       = 100.0;

MAG_Y_COL = 2;   % horizontal axis 1 (Y in body frame)
MAG_Z_COL = 3;   % horizontal axis 2 (Z in body frame)

% ------------------------------------------------------------------ %
%  Load rotation data (full circle of mag readings for calibration)   %
% ------------------------------------------------------------------ %
fprintf('Loading Training Data/calib1_rotate.mat ...\n');
load('Training Data/calib1_rotate.mat');

mag_data  = squeeze(out.Sensor_MAG.signals.values)';   % [N x 3]
time_mag  = out.Sensor_MAG.time;                        % [N x 1]

% Also load gyro to identify a stationary period within this recording
gyro_data = squeeze(out.Sensor_GYRO.signals.values)';
time_gyro = out.Sensor_GYRO.time;

fprintf('Magnetometer samples: %d  over %.2f s\n', length(time_mag), time_mag(end));
fprintf('\n');

% ------------------------------------------------------------------ %
%  STEP 4a — Characterise the raw Lissajous shape                     %
% ------------------------------------------------------------------ %
fprintf('--- Step 4a: Raw magnetometer shape (Y vs Z axes) ---\n');

mag_y_raw = mag_data(:, MAG_Y_COL);
mag_z_raw = mag_data(:, MAG_Z_COL);
mag_x_raw = mag_data(:, 1);

fprintf('Raw magnetometer statistics:\n');
for col = 1:3
    fprintf('  Axis %d:  min = %+.6e T,  max = %+.6e T,  mean = %+.6e T,  std = %.6e T\n', ...
        col, min(mag_data(:,col)), max(mag_data(:,col)), ...
        mean(mag_data(:,col)), std(mag_data(:,col)));
end
fprintf('\n');

% Shape diagnostics without plotting
y_centre_raw = (max(mag_y_raw) + min(mag_y_raw)) / 2;
z_centre_raw = (max(mag_z_raw) + min(mag_z_raw)) / 2;
y_amp_raw    = (max(mag_y_raw) - min(mag_y_raw)) / 2;
z_amp_raw    = (max(mag_z_raw) - min(mag_z_raw)) / 2;
aspect_raw   = y_amp_raw / z_amp_raw;

fprintf('Lissajous shape (Y vs Z, raw):\n');
fprintf('  Centre:        Y = %+.4e T,  Z = %+.4e T\n', y_centre_raw, z_centre_raw);
fprintf('  Semi-axis Y:   %.4e T\n', y_amp_raw);
fprintf('  Semi-axis Z:   %.4e T\n', z_amp_raw);
fprintf('  Aspect ratio (Y/Z):  %.4f  (1.000 = perfect circle)\n', aspect_raw);
if abs(y_centre_raw) > 0.1 * y_amp_raw || abs(z_centre_raw) > 0.1 * z_amp_raw
    fprintf('  Shape: OFFSET ELLIPSE — hard-iron distortion present\n');
else
    fprintf('  Shape: centred (low hard-iron distortion)\n');
end
if aspect_raw < 0.9 || aspect_raw > 1.1
    fprintf('  Shape: ELLIPTICAL — soft-iron distortion present (aspect ratio outside 0.9–1.1)\n');
else
    fprintf('  Shape: approximately circular (soft-iron distortion low)\n');
end
fprintf('\n');

% Also produce the plot (user can view it in MATLAB)
figure;
plot(mag_y_raw * 1e6, mag_z_raw * 1e6, '.', 'MarkerSize', 3);
axis equal;
xlabel('Mag Y (\muT)'); ylabel('Mag Z (\muT)');
title('Raw magnetometer Lissajous — Y vs Z (horizontal axes)');
grid on;
fprintf('  [Figure 1 opened: raw Lissajous — Y vs Z]\n\n');

% ------------------------------------------------------------------ %
%  STEP 4b — Hard-iron offsets                                         %
% ------------------------------------------------------------------ %
fprintf('--- Step 4b: Hard-iron offset computation ---\n');

mag_offset_y = (max(mag_y_raw) + min(mag_y_raw)) / 2;
mag_offset_z = (max(mag_z_raw) + min(mag_z_raw)) / 2;

fprintf('Computed hard-iron offsets:\n');
fprintf('  Offset Y: %+.6e T\n', mag_offset_y);
fprintf('  Offset Z: %+.6e T\n', mag_offset_z);
fprintf('\n');
fprintf('myEKF.m hard-iron offsets:\n');
fprintf('  mag_hard_iron(1) [Y]: %+.6e T\n', MYEKF_HARD_IRON_Y);
fprintf('  mag_hard_iron(2) [Z]: %+.6e T\n', MYEKF_HARD_IRON_Z);
fprintf('\n');

diff_y = abs(mag_offset_y - MYEKF_HARD_IRON_Y);
diff_z = abs(mag_offset_z - MYEKF_HARD_IRON_Z);
% Threshold: 10% of the amplitude (significant relative to the signal)
thr_y = 0.1 * y_amp_raw;
thr_z = 0.1 * z_amp_raw;

fprintf('Differences (verified vs myEKF.m):\n');
fprintf('  Y: %.4e T  (threshold %.4e T)', diff_y, thr_y);
if diff_y > thr_y
    fprintf('  *** DISCREPANCY ***\n');
else
    fprintf('  OK\n');
end
fprintf('  Z: %.4e T  (threshold %.4e T)', diff_z, thr_z);
if diff_z > thr_z
    fprintf('  *** DISCREPANCY ***\n');
else
    fprintf('  OK\n');
end
fprintf('\n');

% ------------------------------------------------------------------ %
%  STEP 4c — Soft-iron scale factors                                   %
% ------------------------------------------------------------------ %
fprintf('--- Step 4c: Soft-iron scale factor computation ---\n');

% Apply hard-iron correction first (using verified offsets)
mag_y_cal = mag_y_raw - mag_offset_y;
mag_z_cal = mag_z_raw - mag_offset_z;

amp_y = (max(mag_y_cal) - min(mag_y_cal)) / 2;
amp_z = (max(mag_z_cal) - min(mag_z_cal)) / 2;
avg_amp = (amp_y + amp_z) / 2;

% calibration.m formula (same as myEKF.m convention):
%   soft_iron_y = avg_amp / amp_y  (multiply to equalise amplitudes)
%   soft_iron_z = avg_amp / amp_z
soft_iron_y = avg_amp / amp_y;
soft_iron_z = avg_amp / amp_z;

% Alternative description: scale_factor = Y_amp / Z_amp (ratio)
scale_ratio = amp_y / amp_z;

fprintf('After hard-iron removal:\n');
fprintf('  Semi-axis Y: %.6e T\n', amp_y);
fprintf('  Semi-axis Z: %.6e T\n', amp_z);
fprintf('  Average amp: %.6e T\n', avg_amp);
fprintf('  Soft-iron scale Y (avg/amp_Y): %.6f\n', soft_iron_y);
fprintf('  Soft-iron scale Z (avg/amp_Z): %.6f\n', soft_iron_z);
fprintf('  Y/Z amplitude ratio:           %.4f  (1.000 = circular)\n', scale_ratio);
fprintf('\n');
fprintf('myEKF.m soft-iron scales:\n');
fprintf('  mag_soft_iron(1) [Y]: %.6f\n', MYEKF_SOFT_IRON_Y);
fprintf('  mag_soft_iron(2) [Z]: %.6f\n', MYEKF_SOFT_IRON_Z);
fprintf('\n');

diff_si_y = abs(soft_iron_y - MYEKF_SOFT_IRON_Y);
diff_si_z = abs(soft_iron_z - MYEKF_SOFT_IRON_Z);
fprintf('Differences:\n');
fprintf('  Y: %.6f', diff_si_y);
if diff_si_y > 0.05
    fprintf('  *** DISCREPANCY (> 0.05) ***\n');
else
    fprintf('  OK\n');
end
fprintf('  Z: %.6f', diff_si_z);
if diff_si_z > 0.05
    fprintf('  *** DISCREPANCY (> 0.05) ***\n');
else
    fprintf('  OK\n');
end

if scale_ratio < 0.9 || scale_ratio > 1.1
    fprintf('  *** Soft-iron distortion significant (ratio outside 0.9–1.1) ***\n');
else
    fprintf('  Soft-iron distortion is minor (ratio within 0.9–1.1)\n');
end
fprintf('\n');

% Show calibrated Lissajous shape
mag_y_full_cal = mag_y_cal * soft_iron_y;
mag_z_full_cal = mag_z_cal * soft_iron_z;
fprintf('After full calibration (hard + soft iron):\n');
fprintf('  Semi-axis Y: %.6e T\n', (max(mag_y_full_cal)-min(mag_y_full_cal))/2);
fprintf('  Semi-axis Z: %.6e T\n', (max(mag_z_full_cal)-min(mag_z_full_cal))/2);
fprintf('  Aspect ratio: %.4f  (target 1.000)\n', ...
    ((max(mag_y_full_cal)-min(mag_y_full_cal))) / ((max(mag_z_full_cal)-min(mag_z_full_cal))));

figure;
plot(mag_y_full_cal * 1e6, mag_z_full_cal * 1e6, '.', 'MarkerSize', 3);
axis equal;
xlabel('Mag Y calibrated (\muT)'); ylabel('Mag Z calibrated (\muT)');
title('Calibrated magnetometer Lissajous — should be a centred circle');
grid on;
fprintf('  [Figure 2 opened: calibrated Lissajous]\n\n');

% ------------------------------------------------------------------ %
%  STEP 4d — Magnetometer noise from stationary window                 %
% ------------------------------------------------------------------ %
fprintf('--- Step 4d: Magnetometer noise level ---\n');

% Find stationary period within calib1_rotate.mat using gyro magnitude
STAT_THRESH = 0.05;  % rad/s
gyro_mag = sqrt(sum(gyro_data.^2, 2));

% Interpolate gyro stationary mask onto mag time axis
gyro_stat_binary = double(gyro_mag < STAT_THRESH);
gyro_stat_interp = interp1(time_gyro, gyro_stat_binary, time_mag, 'nearest', 0);
stat_mask_mag = logical(gyro_stat_interp);

n_stat_mag = sum(stat_mask_mag);
if n_stat_mag < 10
    fprintf('WARNING: fewer than 10 stationary mag samples found in calib1_rotate.mat.\n');
    fprintf('  Falling back to first 1 second of recording for noise estimate.\n');
    stat_mask_mag = time_mag <= (time_mag(1) + 1.0);
    n_stat_mag = sum(stat_mask_mag);
end

fprintf('Stationary mag samples used for noise: %d  (%.2f s)\n', ...
    n_stat_mag, sum(stat_mask_mag) / (1/mean(diff(time_mag))));

mag_stat = mag_data(stat_mask_mag, :);
sigma_mag_y = std(mag_stat(:, MAG_Y_COL));
sigma_mag_z = std(mag_stat(:, MAG_Z_COL));
R_mag_verified = mean([sigma_mag_y^2, sigma_mag_z^2]);

fprintf('\nMag noise (stationary):\n');
fprintf('  Axis Y (col %d): sigma = %.6e T,  variance = %.6e T^2\n', ...
    MAG_Y_COL, sigma_mag_y, sigma_mag_y^2);
fprintf('  Axis Z (col %d): sigma = %.6e T,  variance = %.6e T^2\n', ...
    MAG_Z_COL, sigma_mag_z, sigma_mag_z^2);
fprintf('  R_mag_verified (mean of Y,Z variances): %.6e T^2\n', R_mag_verified);
fprintf('\n');
fprintf('  R_mag in myEKF.m:  %.1f\n', MYEKF_R_MAG);
fprintf('  Ratio (myEKF / verified): %.2e x\n', MYEKF_R_MAG / R_mag_verified);
fprintf('\n');
fprintf('  NOTE: R_mag = 100.0 is explicitly a tuning hack in myEKF.m, not a\n');
fprintf('  measured noise value (comment: "motor EMI corrupts mag during Task 2").\n');
fprintf('  The measured noise is %.2e T^2 — R_mag inflated by %.2e x.\n', ...
    R_mag_verified, MYEKF_R_MAG / R_mag_verified);
fprintf('  Implication: K_mag ≈ 0 → magnetometer is nearly ignored.\n');
fprintf('  Root fix is EMI rejection or mag axis selection, not inflating R.\n');

fprintf('\n=============================================================\n');
fprintf(' Step 4 complete. Paste this output back to Claude.\n');
fprintf('=============================================================\n');
