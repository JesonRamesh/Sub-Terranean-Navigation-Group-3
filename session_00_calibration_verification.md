# Session 0: Calibration Verification — From First Principles

## Purpose
Every constant hardcoded in `myEKF.m` was computed from calibration data by some script.
This session verifies that:
1. The correct mathematical formula was used for each constant
2. The correct data window was used (stationary vs moving)
3. The correct sensor axis is being used for each physical quantity
4. The noise covariance values (R, Q) are grounded in actual sensor statistics, not guesswork

**Do NOT touch `myEKF.m` during this session.**
**Do NOT run `test_ekf.m` during this session.**
This session only reads data and existing scripts, computes ground-truth reference values,
and produces a verified constants report. Changes to `myEKF.m` happen in Sessions 1-5.

---

## Step 0 — Inventory: what files exist?

List every `.m` file in the project directory. For any file with "calib" in the name,
read it in full and summarise in one sentence what it computes.

Also read the current `myEKF.m` and extract EVERY hardcoded numeric constant into a table:
| Constant name | Value | Where in code | What it's supposed to represent |
|--------------|-------|---------------|--------------------------------|
(Fill in the table. Do not skip any constants, including Q, R matrices, thresholds, arena bounds.)

---

## Step 1 — Gyro Bias Verification

### The correct formula
Gyro bias is the mean output of the gyro when the sensor is completely stationary:
```
b_gyro = mean(Sensor_GYRO(stationary_window, axis))
```
The standard deviation of that window gives the gyro noise (used for R_gyro):
```
sigma_gyro = std(Sensor_GYRO(stationary_window, axis))
R_gyro = sigma_gyro^2
```

### What to do
Load `calibration_data_1.mat`. Then:

**1a. Identify the stationary window.**
Plot `norm(Sensor_GYRO, 2)` (magnitude) over time. The stationary period is where this
is close to zero. Print the start and end time of the stationary window in seconds.

**1b. Check which axis is used.**
The robot rotates in the horizontal plane → yaw rate is the vertical (Z) axis of the IMU.
Plot all three gyro axes during a known rotation (use calibration_data_2.mat — the 5 rotation cycles).
The axis with the largest signal during rotation IS the yaw axis. Confirm it matches the axis
currently used in `myEKF.m`. State clearly: "gyro axis N is yaw" and whether this matches the code.

**1c. Compute the correct bias.**
Using only the stationary window identified in step 1a, compute:
```matlab
gyro_bias_verified = mean(Sensor_GYRO(stationary_mask, yaw_axis));
gyro_noise_sigma   = std(Sensor_GYRO(stationary_mask, yaw_axis));
R_gyro_verified    = gyro_noise_sigma^2;
```
Print all three values. Compare `gyro_bias_verified` to the value currently in `myEKF.m` (0.00186 rad/s).
If they differ by more than 0.001 rad/s, flag it as a discrepancy.

**1d. Check stationary window length.**
A 1-second window at 104 Hz = 104 samples. Minimum recommended: 5 seconds = 520 samples.
Print how many samples are in the stationary window. If < 520, note this as a concern.

---

## Step 2 — Accelerometer Bias and Axis Mapping Verification

### The correct formula
On a stationary flat surface, the accelerometer measures gravity only.
The axis aligned with gravity reads ≈ ±9.81 m/s² (sign depends on mounting direction).
The axes perpendicular to gravity read ≈ 0 m/s².

```
accel_bias_i = mean(Sensor_ACCEL(stationary_window, i))   for each axis i = 1,2,3
```

For the EKF, only the HORIZONTAL axes matter for position integration.
The vertical axis (gravity ≈ ±9.81) must NOT be used for velocity integration.

### What to do
Load `calibration_data_1.mat`. Use the same stationary window from Step 1.

**2a. Compute mean and std for all three accelerometer axes:**
```matlab
for i = 1:3
    fprintf('Accel axis %d: mean = %.5f m/s², std = %.5f m/s²\n', ...
        i, mean(Sensor_ACCEL(stationary_mask,i)), std(Sensor_ACCEL(stationary_mask,i)));
end
```
Print the results. Identify which axis has |mean| ≈ 9.81 → this is the VERTICAL axis.
The other two axes (with |mean| ≈ 0) are the horizontal axes.

**2b. Identify the forward axis.**
Load `calibration_data_1.mat`. During the straight-line forward/back motion, the robot
accelerates and decelerates along its forward axis. Plot all three accel axes during the
moving phase. The axis with the largest signal swings during motion is the FORWARD axis.

Print: "Forward acceleration axis is column N" and whether this matches `myEKF.m`.

**2c. Identify the lateral (sideways) axis.**
The remaining horizontal axis is lateral (left/right motion). Mecanum wheels can strafe.
Print: "Lateral acceleration axis is column N".

**2d. Verify the biases in `myEKF.m`.**
Current code uses `accel_bias_x = 9.84855` for axis 1. This confirms axis 1 is vertical.
For the forward axis (identified in 2b), what bias is used in the current code?
If the forward axis bias ≈ 0 but the code uses 9.84855 for that axis — this is a critical bug.
State clearly whether the axis mapping in `myEKF.m` is correct or wrong.

**2e. Compute verified horizontal noise:**
```matlab
sigma_accel_fwd = std(Sensor_ACCEL(stationary_mask, fwd_axis));
R_accel_verified = sigma_accel_fwd^2;
```
Print the value. Compare to `R_accel` currently in `myEKF.m`.

---

## Step 3 — Gyro Scale Factor Verification (using calibration_data_2.mat)

### The correct formula
The robot completes exactly 5 full rotations (5 × 360° = 1800° = 10π radians total).
If the gyro is properly calibrated, integrating the bias-corrected gyro rate over the
entire recording should give exactly 10π radians (or -10π if rotating CW).

```
total_angle = trapz(time, gyro_data(:,yaw_axis) - gyro_bias)
expected    = 5 * 2 * pi   % = 31.416 rad
error_pct   = abs(total_angle - expected) / expected * 100
```

### What to do
Load `calibration_data_2.mat`.

**3a. Integrate the gyro over the full recording:**
```matlab
dt = mean(diff(Sensor_Time));
gyro_corrected = Sensor_GYRO(:, yaw_axis) - gyro_bias_verified;  % use verified value from Step 1
total_angle = cumsum(gyro_corrected) * dt;
```
Plot `total_angle` over time. It should be a monotonically increasing (or decreasing) ramp
that ends near ±10π ≈ ±31.416 rad.

**3b. Print the actual total angle and the error:**
```matlab
fprintf('Total integrated angle: %.4f rad\n', total_angle(end));
fprintf('Expected (5 rotations): %.4f rad\n', 5*2*pi);
fprintf('Error: %.2f%%\n', abs(total_angle(end) - 5*2*pi) / (5*2*pi) * 100);
```

**3c. Interpret:**
- Error < 2%: gyro scale is fine
- Error 2–5%: minor scale error, acceptable
- Error > 5%: significant scale error → a scale factor correction is needed

If error > 5%, compute the correction:
```matlab
scale_factor = (5*2*pi) / total_angle(end);
fprintf('Gyro scale factor correction: %.6f\n', scale_factor);
```
State whether a scale factor correction is currently applied in `myEKF.m`.

---

## Step 4 — Magnetometer Reference Heading Verification

### The correct formula
The magnetometer measures the ambient magnetic field vector. To get heading (yaw angle),
the horizontal components are used:
```
heading_mag = atan2(-Sensor_MAG(:,2), Sensor_MAG(:,1))   % standard 2D compass formula
```
The reference heading (used in the EKF measurement model) must match the INITIAL heading
of the robot at the start of the Task recording — NOT of the calibration recording.

Hard-iron distortion (bias in each axis from permanent magnets) must be removed first:
```
mag_x_corrected = Sensor_MAG(:,1) - mean(Sensor_MAG(:,1))  % rough hard-iron removal
mag_y_corrected = Sensor_MAG(:,2) - mean(Sensor_MAG(:,2))
```
Soft-iron distortion (elliptical distortion) requires fitting an ellipse to the mag data.

### What to do
Load `calibration_data_2.mat` (the rotation data — 5 full cycles gives a full circle of mag readings).

**4a. Plot raw magnetometer data (Lissajous plot).**
```matlab
figure; plot(Sensor_MAG(:,1), Sensor_MAG(:,2), '.');
axis equal; xlabel('Mag X (T)'); ylabel('Mag Y (T)');
title('Raw magnetometer — should be a circle if no hard-iron distortion');
```
Describe the shape: is it a circle (good) or an offset ellipse (hard-iron + soft-iron distortion)?

**4b. Compute hard-iron offsets:**
```matlab
mag_offset_x = (max(Sensor_MAG(:,1)) + min(Sensor_MAG(:,1))) / 2;
mag_offset_y = (max(Sensor_MAG(:,2)) + min(Sensor_MAG(:,2))) / 2;
fprintf('Hard-iron offset X: %.6f T\n', mag_offset_x);
fprintf('Hard-iron offset Y: %.6f T\n', mag_offset_y);
```
Are these offsets currently applied in `myEKF.m`? If not, flag as a bug.

**4c. Compute soft-iron scale factors:**
After removing hard-iron offsets:
```matlab
mag_x_cal = Sensor_MAG(:,1) - mag_offset_x;
mag_y_cal = Sensor_MAG(:,2) - mag_offset_y;
scale_x = (max(mag_x_cal) - min(mag_x_cal)) / 2;
scale_y = (max(mag_y_cal) - min(mag_y_cal)) / 2;
scale_factor_soft = scale_x / scale_y;  % should be 1.0 if circular
fprintf('Soft-iron scale factor (X/Y): %.4f\n', scale_factor_soft);
```
If scale_factor_soft is not between 0.9 and 1.1, soft-iron correction is needed.
Is soft-iron correction currently applied in `myEKF.m`?

**4d. Verify the magnetometer noise level:**
Using the stationary window from Step 1 (or a stationary period from calibration_data_2):
```matlab
sigma_mag = std(Sensor_MAG(stationary_mask, 1:2), [], 1);
R_mag_verified = mean(sigma_mag.^2);
fprintf('Magnetometer noise variance (verified): %.6f T²\n', R_mag_verified);
fprintf('R_mag currently in myEKF.m: %.6f\n', R_mag_current);
```
Note: `R_mag = 100.0` is an extreme tuning value, not a measured noise value.
The measured R should be much smaller. The large value is a workaround for EMI corruption.
Flag this explicitly — the root fix is EMI rejection, not inflating R_mag.

---

## Step 5 — Process Noise Q Matrix Verification

### The correct formula
Q is the process noise covariance — it represents how much the true state can change
unexpectedly between prediction steps, due to unmodelled dynamics or sensor limitations.

For each state, Q_ii should be approximately:
```
Q_x, Q_y  ≈ (position noise per step)²   — typically very small (1e-4 to 1e-6)
Q_theta   ≈ (gyro_noise × dt)²            — = R_gyro × dt²
Q_vx, Q_vy ≈ (accel_noise × dt)²          — = R_accel × dt²
```
where dt = 1/104 ≈ 0.0096s (IMU update interval).

### What to do
Using the noise values computed in Steps 1-2:
```matlab
dt_imu = 1/104;
Q_theta_derived  = R_gyro_verified * dt_imu^2;
Q_vx_derived     = R_accel_verified * dt_imu^2;

fprintf('Derived Q_theta: %.2e\n', Q_theta_derived);
fprintf('Derived Q_vx/vy: %.2e\n', Q_vx_derived);
fprintf('Current Q in myEKF.m: diag([1e-4, 1e-4, 1e-4, 0.1, 0.1])\n');
```

Compare derived values to current Q. If current Q_vx=0.1 but derived Q_vx ≈ 1e-6,
the current Q is ~100,000× too large. This has a direct effect: large Q_vx means the
filter trusts the accelerometer very little, allowing velocity to drift freely from noise.
Explain the implication clearly.

---

## Step 6 — Produce Verified Constants Report

At the end of this session, produce a MATLAB comment block ready to paste into `myEKF.m`:

```matlab
% =========================================================
% VERIFIED CALIBRATION CONSTANTS — Session 0, 2026-03-17
% =========================================================
% Gyro
%   Yaw axis:              column [N]
%   Stationary bias:       [value] rad/s  (computed from [N] samples, [T] seconds)
%   Scale factor:          [value]        (error: [%]% vs 5-rotation ground truth)
%   Noise sigma:           [value] rad/s
%   R_gyro (verified):     [value] (rad/s)²
%
% Accelerometer
%   Vertical axis:         column [N] (bias ≈ 9.81 m/s² — do NOT use for integration)
%   Forward axis:          column [N] (bias: [value] m/s²)
%   Lateral axis:          column [N] (bias: [value] m/s²)
%   Noise sigma (fwd):     [value] m/s²
%   R_accel (verified):    [value] (m/s²)²
%
% Magnetometer
%   Hard-iron offset X:    [value] T
%   Hard-iron offset Y:    [value] T
%   Soft-iron scale (X/Y): [value]  ([OK / CORRECTION NEEDED])
%   Noise sigma:           [value] T
%   R_mag (measured):      [value] T²
%   R_mag (current tuned): 100.0   (NOT a noise measurement — EMI workaround)
%
% Process noise Q (derived from sensor noise):
%   Q_x = Q_y:   [value]
%   Q_theta:     [value]
%   Q_vx = Q_vy: [value]
%
% Discrepancies found (must be resolved before Sessions 1-5):
%   [list each discrepancy between verified values and current myEKF.m values]
% =========================================================
```

Fill in every field. List every discrepancy at the bottom. This report is the ground truth
that Sessions 1-5 will use to make informed changes.

---

## What a "pass" looks like for this session
- Every constant in `myEKF.m` has a verified value computed from data
- The correct gyro axis for yaw has been confirmed
- The vertical vs horizontal accelerometer axes have been confirmed
- Hard-iron offsets for magnetometer have been computed and compared to code
- Q matrix values have been derived and compared to current values
- A complete discrepancy list exists

Only after this session is complete should Sessions 1-5 proceed.
