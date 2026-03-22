# Session 4: Re-estimate Gyro Bias from Task 2 Data

## What this session does
The stationary calibration bias (0.00186 rad/s) is measured with the robot still.
During Task 2 motion, the effective gyro bias is ≈ 0.031 rad/s — 17× larger due to
Mecanum wheel vibration coupling into the gyro sensor.

If the Task 2 dataset contains a stationary period at the very start (robot sitting still
before the run begins), we can compute a better bias estimate directly from that data.

## What to do

### Step 1 — Inspect Task 2 gyro data before any changes
Write a small diagnostic script (NOT inside myEKF.m) called `check_task2_bias.m` that:
1. Loads the Task 2 .mat file
2. Plots gyro axis 1 for the first 5 seconds
3. Plots the magnitude of GT velocity (sqrt(vx² + vy²)) for the first 5 seconds
4. Prints the mean gyro reading for t < 2s and t < 5s

Run it and report:
- Is there a visibly stationary period at the start?
- What is mean(gyro(:,1)) during t < 2s?
- Does the GT velocity confirm the robot is stationary during this period?

### Step 2 — Interpret the result
Two scenarios:
(A) Stationary period exists + mean gyro reading differs from 0.00186:
    → Use this value as the Task 2 gyro bias. Potentially large improvement.
(B) No stationary period, or mean ≈ 0.00186:
    → The bias can't be improved from data alone. Session 5 (online estimation) is needed.

### Step 3 — If scenario A: implement in myEKF.m
Add a bias estimation block at the very start of myEKF.m, BEFORE the main filter loop:
```matlab
% Estimate gyro bias from first 2 seconds of data (if robot is stationary)
stationary_mask = out.Sensor_Time <= 2.0;
gyro_bias_x = mean(out.Sensor_GYRO(stationary_mask, 1));
fprintf('Estimated gyro bias: %.5f rad/s\\n', gyro_bias_x);
```
This runs once at startup, not inside the loop.

### Step 4 — Run test_ekf.m and report ALL metrics
Note: this change should NOT affect Task 1 (same bias estimation logic applies).

### Step 5 — Update DEBUG_LOG.md
