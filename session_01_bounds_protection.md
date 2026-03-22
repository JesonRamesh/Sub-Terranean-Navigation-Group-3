# Session 1: Bounds Protection in calculate_expected_tof

## What this session does
Adds a safety guard to the ToF measurement model so that when the EKF position estimate
accidentally drifts outside the arena walls (which happens during Task 2 when heading is wrong),
the function does not return negative or zero predicted distances. A negative predicted distance
fed into the Kalman update step causes a numerically catastrophic update that throws the state
estimate to ±10,000m — this is the divergence visible in test_ekf.m's right subplot.

## Why this is the FIRST fix (not the last)
All other fixes (velocity decay, gyro bias, magnetometer tuning) reduce HOW MUCH the state
drifts outside the arena. But none of them fully prevent it — especially early in debugging.
Without bounds protection, a single bad EKF update can cause irreversible divergence that
masks whether other fixes are actually working.

Think of this as putting a safety net under a tightrope walker. It doesn't improve their skill,
but it prevents a single stumble from ending the experiment entirely.

## What to do

### Step 1 — Read, don't touch yet
Read `myEKF.m` and find the function `calculate_expected_tof` (or equivalent).
Show me the current implementation in full.

### Step 2 — Explain before changing
Before writing any code, write a 3-sentence plain-English explanation of:
(a) how the function currently computes the expected ToF distance from robot state X
(b) what happens mathematically when x or y is outside the arena bounds
(c) exactly where the fix will be inserted

### Step 3 — Implement bounds protection
Add the following logic to `calculate_expected_tof`:
```matlab
% After computing predicted distances for each wall:
distances(distances <= 0) = inf;      % invalid: behind wall or outside arena
[h_x, wall_idx] = min(distances);
if isinf(h_x)
    % EKF position is outside arena — skip this ToF update entirely
    H_tof = zeros(1, length(X));
    return;
end
```
Show the full modified function before applying it.

### Step 4 — Run test_ekf.m
Run test_ekf.m on BOTH Task 1 and Task 2 data.
Report ALL of these metrics:
- Task 1 RMSE (must remain ≤ 0.05m)
- Task 2 RMSE
- Task 2 max heading error
- Task 2 RMS heading error
- Task 2 final heading error
- Does the right subplot still diverge to ±10,000m?

### Step 5 — Update DEBUG_LOG.md
Append a new section "2.5 Change 5: Bounds Protection" to DEBUG_LOG.md with
the before/after metrics table and a one-paragraph conclusion.

## Expected outcome
- Task 1 RMSE: unchanged (~0.047m)
- Task 2 RMSE: may improve slightly or stay similar
- Right subplot divergence: should be significantly reduced or eliminated
- This fix alone will NOT bring Task 2 RMSE to 0.3m — it just prevents catastrophic blowup
