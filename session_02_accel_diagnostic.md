# Session 2: Diagnostic — Disable Accelerometer

## What this session does
Tests whether the accelerometer is helping or hurting Task 2 performance by temporarily
setting ax = 0 and ay = 0 in the prediction step. This is a DIAGNOSTIC, not a final fix.

## Why this matters
The debug log shows `accel_data(:,1)` has a bias of 9.84855 m/s² — that's approximately g (9.81).
This means axis 1 is the VERTICAL axis of the sensor. After subtracting the bias, you're left
with near-zero signal plus wheel vibration noise. You are NOT measuring horizontal forward motion.

The result: `vy_est` reaches -4 m/s in Task 2 (physically impossible for this robot).
This runaway velocity then gets integrated into position with no ToF corrections to stop it
(because the ToF gate rejects updates when heading is wrong).

The accelerometer may be providing zero useful signal and substantial harmful noise.

## What to do

### Step 1 — Check axis mapping first
Before changing anything, read the current `myEKF.m` predict step.
Answer these questions from the code:
(a) Which column of `Sensor_ACCEL` is used for vx integration?
(b) Which column is used for vy integration?
(c) What bias values are subtracted from each?
(d) Does the predict step transform from sensor frame to world frame using theta?

If (d) is NO — i.e., there is no rotation matrix applied — then the accelerometer axes are
being used in the wrong reference frame entirely. Explain this to me in plain English if so.

### Step 2 — Explain before changing
In 2-3 sentences: why would setting ax=ay=0 potentially IMPROVE Task 2 RMSE?
(The answer: the EKF would rely on ToF for position updates and gyro for heading,
which is exactly what Task 1 already does successfully.)

### Step 3 — Make the diagnostic change
Add these two lines in the predict step, clearly marked as a diagnostic:
```matlab
% DIAGNOSTIC: disable accelerometer to test if it harms Task 2
ax = 0;
ay = 0;
```
Also set Q_vx and Q_vy small (e.g., 1e-6) to prevent velocity from drifting randomly.

### Step 4 — Run test_ekf.m on both tasks
Report:
- Task 1 RMSE
- Task 2 RMSE
- Task 2 max/RMS/final heading error
- Does vy_est still reach -4 m/s? (run debug_task2.m)

### Step 5 — Interpret the result
Two possible outcomes:
(A) Task 2 RMSE IMPROVES → accelerometer was harmful. Conclusion: don't use it. Move to Session 3.
(B) Task 2 RMSE GETS WORSE → accelerometer was providing some useful signal. Keep it but fix the axis mapping.

Report which outcome occurred and what it means for the next session.

### Step 6 — Update DEBUG_LOG.md
Append "2.6 Change 6: Accelerometer Disabled (Diagnostic)" with results + interpretation.
