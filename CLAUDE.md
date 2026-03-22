# COMP0217 Sub-Terranean Navigation — EKF Project
**Branch:** `fix-task2-from-helitha`
**Course:** UCL COMP0217 Sensor Fusion

---

## Project Goal
Implement `myEKF.m`: a single MATLAB function with signature:
```matlab
[X_Est, P_Est] = myEKF(out)
```
Estimates 2D robot position (x, y) and yaw using IMU + ToF sensors. No GPS.

---

## Hard Constraints (never violate these)
- ONE submission file only: `myEKF.m`
- No external file calls, no hardcoded paths in `myEKF.m`
- The same `myEKF.m` code must work for BOTH Task 1 and Task 2
- Helper functions may be defined inside `myEKF.m` and called from within it
- Output: `X_Est` (state estimates over time), `P_Est` (covariance over time)

---

## Data Format (Simulink .mat files — CRITICAL)
- ALL signals logged at exactly **200 Hz**, dt = 0.005s fixed
- GT_rotation first 2 samples are **zero quaternions** (PhaseSpace lock-on delay)
  → Always use first valid quaternion (norm > 0.9) for theta0
- Quaternion yaw formula gives correct magnitude but **wrong sign** for this mounting
  → theta0 = wrapToPi(atan2(...standard formula...) + pi)
- GT_position columns: col1=arena X, col2=arena Y, col3=height (~0.237m constant)

---

## State Vector (5 states)
```
X = [x; y; theta; vx; vy]
```
- x, y: position in metres (origin = centre of arena)
- theta: yaw angle in radians (robot faces +Y at theta=+pi/2 when starting)
- vx, vy: velocity in body frame (m/s)

---

## Sensors & Update Rates
| Variable | Rate | Sensor | Units |
|----------|------|--------|-------|
| `Sensor_GYRO(:,1)` | 104 Hz (logged at 200Hz) | ISM330DHCX | rad/s (yaw axis confirmed) |
| `Sensor_ACCEL(:,3)` | 104 Hz (logged at 200Hz) | ISM330DHCX | m/s² (**col3=forward**, col2=lateral, col1=vertical/gravity) |
| `Sensor_MAG` | 50 Hz | IIS2MDC | Tesla (horizontal axes: col2, col3) |
| `Sensor_ToF1/2/3(:,1)` | 10 Hz | VL53L1X | metres |
| `GT_position` | 200 Hz | PhaseSpace | metres |
| `GT_rotation` | 200 Hz | PhaseSpace | quaternion [W,X,Y,Z] |

**ToF sensor layout (CONFIRMED by data verification):**
- ToF1: LEFT-facing (alpha = pi/2)   ← was wrongly labelled Forward
- ToF2: FORWARD-facing (alpha = 0)   ← was wrongly labelled Left
- ToF3: RIGHT-facing (alpha = -pi/2) ← confirmed correct

---

## Arena
- Square walled arena, origin at centre
- Confirmed bounds: `x ∈ [-1.22, 1.22]`, `y ∈ [-1.22, 1.22]` (244cm × 244cm)
- Some walls have holes → ToF outlier rejection needed
- Robot starts against south wall (y ≈ -0.933), facing +Y (theta = pi/2)

---

## Tasks
| Task | Duration | Motion | RMSE Target | Current |
|------|----------|--------|-------------|---------|
| Task 1 | 14s | Straight line forward + back | ≤ 0.05m | 0.172m (Helitha baseline 0.030m — regression exists) |
| Task 2 | 52s | Rectangular circuit (~360° rotation) | ≤ 0.30m | 0.684m (Helitha baseline 1.02m — improvement of 0.336m, H_tof(3)=0 applied) |

---

## Confirmed Calibration Constants (all verified Session 0 + Session 4d)
```matlab
dt               = 0.005;          % Fixed 200Hz Simulink log rate
gyro_bias_x      = 0.00186;        % rad/s — verified Session 0, col 1
accel_bias_fwd   = -0.396;         % m/s² — col3 (forward), verified Session 0
accel_bias_y     = 0.07485;        % m/s² — col2 (lateral)
accel_clip       = 2.0;            % m/s² — vibration spike clipping
mag_hard_iron    = [-3.705e-05, 4.415e-05];  % [col2, col3] offsets (T)
mag_soft_iron    = [1.0958, 0.9196];          % [col2, col3] scale factors
arena_bounds     = x±1.22, y±1.22            % metres
alpha_tof1       = pi/2;           % ToF1 faces LEFT
alpha_tof2       = 0;              % ToF2 faces FORWARD
alpha_tof3       = -pi/2;          % ToF3 faces RIGHT
Q                = diag([1e-4, 1e-4, 1e-4, 0.1, 0.1])
R_mag            = 100.0;          % EMI workaround — not a noise measurement
R_tof            = 0.01;
gamma_threshold  = 9.0;
```

**Theta0 — current code (Session 4d fix, slight angle error remains):**
```matlab
quat_norms = sqrt(sum(gt_rot.^2, 2));
first_valid_idx = find(quat_norms > 0.9, 1);
theta0 = wrapToPi(atan2(2*(qw*qz+qx*qy), 1-2*(qy^2+qz^2)) + pi);
% Gives theta0 ≈ 88.6 deg. True value is pi/2 = 90 deg exactly.
% Session 5a must hardcode theta0 = pi/2 (robot perpendicular to south wall).
```

---

## Resolved Root Causes
| Bug | Session | Fix | RMSE impact |
|-----|---------|-----|-------------|
| Wrong accel axis (col1 vertical used as forward) | S2 | switched to col3 | neutral (masked) |
| Bounds protection missing in ToF model | S1 | distances<=0 → inf | prevented divergence |
| Arena bounds wrong (y_min=-2.16) | S4b | corrected to ±1.22 | geometry corrected |
| Function signature (3 outputs → 2) | S4b | removed GT output | submission compliance |
| GT rotation zero quaternions at t=0 | S4d | find first norm>0.9 | heading correct |
| Quaternion sign convention wrong | S4d | +pi correction | heading correct |
| ToF1/ToF2 mounting angles swapped | S4d | swapped alpha values | **RMSE 0.996→0.172m T1** |
| Simulink 200Hz logging misunderstood | S4d | dt=0.005 fixed | integration correct |
| H_tof(3)=0 (decouple ToF from heading) | S5d | one line | **0.836→0.684m T2** |

---

## Remaining Root Causes

### A — Motion-Induced Gyro Bias [CONFIRMED, HIGH PRIORITY]
- Task 2 theta spikes at turns (visible in state plot t=20-30s)
- Effective bias during motion ≈ 0.031 rad/s → ~15° drift per straight leg
- Cannot be calibrated offline — only present when motors running
- **Fix: Session 5b — 6-state EKF with online gyro bias as 6th state**

### B — Theta0 lateral offset [LOW PRIORITY, QUICK FIX — Session 5a]
- Mean X error Task 1 = -0.139m caused by theta0=88.6° instead of 90°
- Robot confirmed perpendicular to south wall → true theta0 = pi/2
- cos(88.6°)=0.024 creates small sideways drift that accumulates
- **Fix: hardcode theta0 = pi/2 in Session 5a**

---

## Session 5 Plan — Two Sub-Steps

### Session 5a (do first — 1 line change, quick win)
Change theta0 initialisation from quaternion-derived value to hardcoded:
```matlab
% Replace the entire theta0 block with:
theta0 = pi/2;  % Robot starts perpendicular to south wall, facing +Y
```
Expected: Task 1 RMSE drops from 0.172m to ~0.03-0.05m.
Task 2 RMSE may improve slightly.
DO NOT proceed to 5b until 5a is confirmed working.

### Session 5b (main — 6-state EKF)
Extend state vector: X = [x; y; theta; vx; vy; b_gyro]
- b_gyro initialised at stationary value: 0.00186 rad/s
- Prediction: omega = gyro_data(k,1) - X(6)  (use state, not constant)
- F matrix extended to 6×6: ∂b_gyro/∂b_gyro = 1, all other new entries = 0
- Q extended to 6×6: Q[6,6] = 1e-6 (bias changes very slowly)
- P extended to 6×6: P[6,6] = (0.005)² initial bias uncertainty
- H matrices for ToF and mag stay same width but pad with zero in col 6
Expected: Task 2 RMSE drops from 1.084m toward 0.30m target.

---

## Session 5e — Reduce R_mag to Give Heading Anchor During Turns
Change R_mag from 100.0 to 10.0. One line change only.

**Hypothesis:** The magnetometer can provide ~37° correction during turn 1 if trusted more. Motor EMI is a concern but worth testing at R_mag=10 before lower values.

**Protocol:**
- If Task 2 RMSE improves AND yaw max drops below 90°: reduce further to R_mag=5
- If Task 2 gets worse: revert to R_mag=100
- Test both tasks after each change

**Before:** `R_mag = 100.0;`
**After:**  `R_mag = 10.0;`

---

## Testing Infrastructure
| File | Purpose |
|------|---------|
| `test_ekf.m` | Load task data, run EKF, compute RMSE + yaw errors, plot trajectory |
| `debug_task2.m` | Task 2 diagnostics (note: arena bounds inside still need updating to ±1.22) |
| `debug_ekf.m` | Task 1 diagnostics |
| `sensor_diagnostic.m` | Sensor analysis |

**Success criteria per sub-step:**
- 5a: Task 1 RMSE ≤ 0.05m, Task 2 unchanged or better
- 5b: Task 2 RMSE ≤ 0.30m, Task 1 still ≤ 0.05m

---

## Session Discipline Rules
1. One fix per session. Do not bundle 5a and 5b.
2. Before changing `myEKF.m`: explain in plain English what the fix does and why.
3. Before changing `myEKF.m`: show exact lines being changed (before/after).
4. After changing `myEKF.m`: run `test_ekf.m` on BOTH tasks. Report all metrics.
5. If Task 1 RMSE exceeds 0.05m after 5a: revert and diagnose before continuing.
6. Append results to `DEBUG_LOG.md` with: change, before/after table, conclusion.
7. Never silently make multiple changes. Explain ALL parts before changing ANY.
8. For Session 5b specifically: show the full extended F, Q, P matrices before applying.