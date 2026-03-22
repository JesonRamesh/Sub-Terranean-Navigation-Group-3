# Session 3: Velocity Decay Model (Prevent vy Runaway)

## What this session does
Adds a velocity decay term to the EKF prediction step so that `vx` and `vy` cannot
accumulate to physically impossible values (±4 m/s). Without ToF corrections (which are
frequently gated out during Task 2 due to heading errors), an unconstrained velocity state
integrates noise indefinitely.

## Only run this session if:
- Session 2 showed accelerometer is harmful (ax=ay=0 is already applied), OR
- Session 2 showed accelerometer is useful but vy still drifts to ±2+ m/s

## The physics intuition
This robot is indoors on a flat surface. It stops when the motors stop. There is no mechanism
for it to continue accelerating forever. A velocity decay model reflects this physical reality:
in the absence of measured acceleration, the best prior estimate is that velocity is trending
toward zero.

The mathematical form is:
```matlab
X_pred(4) = X(4) * exp(-dt / tau) + ax * dt;  % vx
X_pred(5) = X(5) * exp(-dt / tau) + ay * dt;  % vy
```
where `tau` is the time constant in seconds (e.g., tau=2.0 means velocity halves every ~1.4s).

## What to do

### Step 1 — Show current prediction step
Read `myEKF.m` and show the full prediction step (X_pred computation).
Highlight the exact lines for vx and vy integration.

### Step 2 — Explain the change
Show BOTH the old and new velocity update lines side by side and explain:
- What happens to vx/vy over time if no acceleration is measured (old vs new)
- How tau affects the decay rate
- Why tau=2.0s is a reasonable starting value for this robot

### Step 3 — Also update the F (Jacobian) matrix
The F matrix (∂f/∂X) must be updated to match. The decay term changes the partial
derivative of vx w.r.t. vx from 1.0 to exp(-dt/tau).
Show the Jacobian change before applying it.

### Step 4 — Try tau = 2.0 first
Run test_ekf.m. Report:
- Task 1 RMSE
- Task 2 RMSE
- Does vy_est still reach ±2 m/s? (check with debug_task2.m)

### Step 5 — If Task 2 RMSE does not improve, try tau = 5.0
A longer tau is more conservative (slower decay). Report results.

### Step 6 — Update DEBUG_LOG.md
