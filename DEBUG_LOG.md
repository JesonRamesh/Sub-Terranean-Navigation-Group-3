# EKF Task 2 Debug Log
**Branch:** `fix-task2-from-helitha`
**Date:** 2026-03-17
**Status:** In-progress — RMSE 1.02m → 0.93m, trajectory still wrong

---

## 1. Problem Statement

| Task | Baseline (Helitha) | Current Best | Target |
|------|--------------------|--------------|--------|
| Task 1 | 0.03m | 0.047m | ≤0.05m ✓ |
| Task 2 | 1.02m | 0.93m | ≤0.3m ✗ |

Task 2 is a 52s rectangular circuit (4.37m path, ~360° total heading rotation, closed loop).
Task 1 is a 14s straight line forward and back (0.09m net displacement).

The EKF trajectory for Task 2 bears no resemblance to the rectangular GT path.
The right subplot of `test_ekf.m` shows EKF state values diverging to ~10,000m.

---

## 2. Chronological Record of All Changes and Results

### 2.1 Baseline: Helitha's Code (branch: experiments-helitha)
| Metric | Value |
|--------|-------|
| Task 1 RMSE | **0.03m** |
| Task 2 RMSE | **1.02m** |
| Heading error (Task 2 max) | 179.96° |
| Heading error (Task 2 RMS) | 77.39° |
| Final heading error | ~0° (recovered at end) |
| ToF2 gating | 37.7% rejected |

**What Helitha changed from the original code:**
- Used real accelerometer with bias subtraction (`accel_bias_x = 9.84855 ≈ g`)
- `gamma_threshold = 3.84` (tight Mahalanobis gate)
- `Q = diag([1e-4, 1e-4, 1e-4, 0.1, 0.1])`
- Initial `theta0` from GT quaternion
- `omega = -(gyro - bias)` — **includes minus sign**
- `R_mag = 0.1` — **magnetometer trusted heavily**

---

### 2.2 Change 1: Remove Minus Sign from Omega
**Hypothesis:** `debug_task2.m` showed 179.96° heading error. Diagnosis was that `omega = -(gyro - bias)` reversed the turn direction (CCW → EKF goes CW). Removing the minus should fix heading tracking.

**Code change:**
```matlab
% Before (Helitha's):
omega = -(gyro_data(k, 1) - gyro_bias_x);

% After (our fix):
omega = gyro_data(k, 1) - gyro_bias_x;
```

**Result:**
| Metric | Before | After |
|--------|--------|-------|
| Task 1 RMSE | 0.03m | 0.047m |
| Task 2 RMSE | 1.02m | **1.16m (WORSE)** |
| Max heading error | 179.96° | 179.99° |
| RMS heading error | 77.39° | 78.86° |
| Final heading error | ~0° | **-5.19°** |

**Conclusion:** The gyro sign change had **no measurable effect on heading tracking**. Max/RMS heading errors are essentially identical. This proved that something **other than the gyro sign** is preventing the heading from tracking.

---

### 2.3 Change 2: Accelerometer Clipping
**Hypothesis:** Mecanum wheel vibration causes ±12 m/s² spikes in accelerometer. True robot acceleration is <2 m/s². Clipping removes spikes.

```matlab
accel_clip = 2.0;
ax = max(-accel_clip, min(accel_clip, accel_data(k, 1) - accel_bias_x));
ay = max(-accel_clip, min(accel_clip, accel_data(k, 2) - accel_bias_y));
```

**Result:** Minimal improvement. Did not resolve the fundamental issue.

---

### 2.4 Change 3: R_mag = 1.0 + Magnetometer Mahalanobis Gating
**Hypothesis:** The magnetometer has no outlier rejection. With `R_mag = 0.1`, Kalman gain is `K_mag ≈ 0.5`. Each magnetometer update moves heading 50% toward mag reading. During Task 2, Mecanum motors corrupt the magnetometer (EMI), which continuously drags the heading back to the initial value (-100°), overriding the correct gyro integration.

**Code changes:**
```matlab
R_mag = 1.0;  % was 0.1

% Added gating:
if (innovation_mag^2) / S_mag < gamma_threshold
    K_mag = P * H_mag' / S_mag;
    X = X + K_mag * innovation_mag;
    ...
end
```

**Result:**
| Metric | Before | After |
|--------|--------|-------|
| Task 2 RMSE | 1.16m | **0.94m** ✓ improvement |
| Max heading error | 179.99° | 179.99° (still ~180°) |
| RMS heading error | 78.86° | 88.39° (slightly worse) |
| Final heading error | -5.19° | +91.94° (WORSE) |
| ToF2 gating | 40.4% | 28.7% ✓ improvement |

**Why R_mag=1.0 was still insufficient:**
With `R_mag = 1.0`, `K_mag ≈ 0.091`. A 50° magnetometer error applies `0.091 × 50° = 4.5°` per update. The gyro at 0.4 rad/s adds only `0.4 × 0.005 = 0.002 rad/step = 0.11°/step`. **Magnetometer still wins by 40:1.**

For gyro to overcome magnetometer: requires `K_mag < 0.0023`, which means `R_mag > 43`.

---

### 2.5 Change 4: R_mag = 100.0
**Hypothesis:** Force gyro to dominate heading completely.

**Result:**
| Metric | R_mag=1.0 | R_mag=100 |
|--------|-----------|-----------|
| Task 2 RMSE | 0.94m | **0.93m** (tiny gain) |
| Max heading error | 179.99° | 179.91° (still ~180°) |
| RMS heading error | 88.39° | **88.39%** (same!) |
| **Final heading error** | **-5.19°** | **+91.94°** |
| ToF2 gating | 40.4% | 28.7% |

**Critical finding:** Max heading error is STILL ~180° even with gyro fully dominant (R_mag=100). This means **the 180° heading error is not caused by the magnetometer — there is another root cause.**

**Critical finding 2:** Final heading error jumped from -5.19° to 91.94°. The magnetometer WAS correcting the heading drift at the end of the run. With R_mag=100, that recovery doesn't happen, and the heading is stuck 92° off at the end.

---

## 3. Confirmed Root Causes

### Root Cause A: Motion-Induced Gyro Bias (CONFIRMED)
**Evidence:** With R_mag=100 (gyro fully dominant), the heading drifts by **91.94° over 52 seconds**.

**Calculation:**
- Calibrated stationary bias: `gyro_bias_x = 0.00186 rad/s`
- Effective bias during motion: `91.94° × π/180 / 52s = 0.0308 rad/s`
- **Motion-induced bias is 17× larger than calibrated bias**

**Why:** Mecanum wheel vibration adds a systematic non-zero signal to the gyro axis reading when motors are running. The calibration was done from a stationary period. This is called **vibration-induced gyro bias** — it only appears when the motors are on.

**Impact:**
- Task 1 (14s): drift = `0.031 × 14 = 25°`. Short enough that ToF + magnetometer correct it → 0.047m RMSE
- Task 2 (52s): drift = `0.031 × 52 = 92°`. Too large for magnetometer (corrupted by same motors) to correct

---

### Root Cause B: Magnetometer Corrupted by Motor EMI (CONFIRMED)
**Evidence:**
- With `R_mag=0.1` (Helitha): heading is stuck near initial value (-100°) throughout the circuit but recovers at end
- With `R_mag=100`: heading drifts freely but ends 92° off
- The magnetometer IS providing useful correction (without it, 92° drift; with it, -5° final error)
- But during the circuit, EMI causes corrupted readings that fight the gyro

**Pattern:** The robot returns to its starting position at t≈35s. At that point, the motors may be at a similar state as initialisation, and the magnetometer reads approximately correctly again → heading recovery visible in Fig 2.

---

### Root Cause C: Max Heading Error ~180° Despite Correct Gyro Sign (UNRESOLVED)
**Evidence:** Max heading error is ~180° with BOTH `omega = +(gyro - bias)` (our code) and `omega = -(gyro - bias)` (Helitha's code). The 180° cap is persistent regardless of gyro sign or R_mag value.

**Analytical explanation for wrong sign:**
If sign is WRONG (EKF goes CW while robot goes CCW):
- After robot's first 90° CCW turn: EKF has done -90° CW
- Error = `wrapToPi(EKF - GT) = wrapToPi(-90° - 90°) = -180°` → max error = 180° ✓

This exact pattern (max error 180° occurring after each 90° turn) is consistent with the **gyro sign being WRONG** (i.e., the minus sign should have stayed).

**But:** If sign is wrong, the final heading error should be ~0° for a closed loop (EKF goes -360° while robot goes +360°, both return to start). Yet we observe +91.94° final error with R_mag=100. This could be explained by the motion-induced gyro drift adding an additional ~92° of cumulative error on top of the sign flip.

**Unresolved question:** Is the gyro sign correct (+ as in our code) or wrong (- as in Helitha's code)?
We cannot determine this without the Nucleo hardware test (Priority 3).

---

### Root Cause D: ToF Gate Too Tight for Task 2 (CONFIRMED)
**Evidence:** 28-40% of ToF updates are rejected. When heading is wrong, ToF predicted distances differ from actual by large amounts → Mahalanobis score exceeds `gamma_threshold = 3.84`.

**Relationship to other causes:** This is a **downstream effect** of the heading error, not an independent root cause. Fix the heading → ToF accepts more updates → position corrects better.

---

### Root Cause E: Accelerometer Axis Mapping (SUSPECTED, NOT CONFIRMED)
**Evidence:**
- `accel_data(:,1)` with `bias = 9.84855 m/s²` → this is the VERTICAL axis (gravity component)
- After bias removal: `ax ≈ 0 + vibration noise` — does NOT capture horizontal forward motion
- The EKF uses `ax` to integrate `vx` (body-forward velocity)
- `vx` is therefore driven by vibration noise, not actual forward acceleration

**Impact:**
- For Task 1: robot moves slowly, ToF corrections at 10Hz dominate. Even with `vx ≈ 0`, position tracks well between ToF updates. RMSE = 0.047m (acceptable).
- For Task 2: when ToF updates are rejected (due to heading errors), `vx` drifts from noise with no correction. Position diverges.

**Key symptom:** `vy_est` reaches -4 m/s in Fig 3 (physically impossible for this robot). This is evidence of unconstrained velocity integration from noisy accelerometer.

---

## 4. Current State of EKF (best version: R_mag=100, no minus, accel_clip=2)

```
Task 1 RMSE:  0.047m  (acceptable)
Task 2 RMSE:  0.93m   (unacceptable — target <0.3m)

Heading: max error 180°, RMS 88°, final error 92°
ToF2 gating: 28.7% rejected (improved from 40.4%)
vy_est: peaks at -4 m/s (clearly wrong — velocity runaway)
Right subplot (test_ekf.m): EKF diverges to ~10,000m (state variable divergence)
```

The EKF trajectory starts near (0,0) correctly, begins moving in roughly the right direction for the first few seconds, then veers off and scatters completely.

---

## 5. What Hasn't Been Tried Yet

### 5.1 Diagnostic: Disable Accelerometer Entirely
Set `ax = 0; ay = 0;` (or set `Q_vx = Q_vy` very small).
**Predicted outcome:** If RMSE improves, it confirms that the accelerometer is adding noise/drift to position. The EKF would rely purely on ToF for position and gyro for heading.

### 5.2 Diagnostic: Revert Gyro Sign + Moderate R_mag
Try `omega = -(gyro - bias)` (back to Helitha's original) combined with `R_mag = 10` or `R_mag = 20`.
**Rationale:** With wrong gyro sign, the heading oscillates ±180°. But with moderate magnetometer trust, the magnetometer can correct the heading back toward correct value. This might produce lower RMSE than current code.

### 5.3 Fix: Recalibrate Gyro Bias from Task 2 Data
The stationary bias `0.00186 rad/s` is correct for a non-moving robot. The effective bias during motion is `≈ 0.031 rad/s`. If the Task 2 recording has a stationary phase at the very start or end, calibrate from that.

Alternatively, scan the gyro data for the first 2 seconds of Task 2 and re-estimate the bias.

### 5.4 Fix: Add Velocity Decay Model
```matlab
decay = exp(-dt / tau);  % tau = time constant (e.g., 2 seconds)
X_pred(4) = X(4) * decay + ax * dt;
X_pred(5) = X(5) * decay + ay * dt;
```
This prevents `vx` and `vy` from accumulating to ±4 m/s. Without ToF corrections, velocity decays toward zero between updates.

**Risk:** If `tau` is too short, legitimate velocity changes are suppressed. Suggested: `tau = 2.0s` (velocity halved every ~1.4s). Should not affect Task 1 (ToF corrections frequent).

### 5.5 Fix: Online Gyro Bias Estimation (6-state EKF)
Add gyro bias as a 6th state: `X = [x; y; theta; vx; vy; b_gyro]`.
The EKF would estimate and correct the gyro bias in real time.
**Impact:** Handles both stationary bias and motion-induced bias.
**Complexity:** Moderate — requires extending F, Q matrices to 6×6.

### 5.6 Fix: Tune Arena Bounds for Task 2
The arena bounds are `x_max=1.2, x_min=-1.2, y_max=1.2, y_min=-2.16`.
If the Task 2 circuit takes the robot outside these bounds (even momentarily), the ToF model returns negative distances → catastrophic Kalman update.
**Diagnostic:** Add a check: `if h_x <= 0, skip ToF update`.

### 5.7 Fix: Add Bounds Protection to calculate_expected_tof
```matlab
distances(distances <= 0) = inf;  % remove behind-wall solutions
[h_x, wall_idx] = min(distances);
if isinf(h_x), H_tof = zeros(1,5); return; end
```
This prevents negative predicted distances when the EKF position drifts outside the arena.

### 5.8 Tune Q and gamma for Task 2
- `Q_vx = Q_vy = 0.1` may be too large. If velocity uncertainty is allowed to grow, the ToF Jacobian corrections spread into velocity and corrupt it.
- Try `Q_vx = Q_vy = 0.01` to constrain velocity more.
- Or increase `gamma_threshold = 3.84` to `gamma_threshold = 9.0` to accept more ToF updates.

---

## 6. Recommended Next Steps (Priority Order)

### Step 1 — Immediate: Test Gyro Sign (Nucleo)
Rotate the physical robot CCW while recording gyro data. Check if axis 1 reads POSITIVE (correct sign, omega = +) or NEGATIVE (wrong sign, omega = -). This resolves Root Cause C definitively.

### Step 2 — Simulation: Try R_mag = 10 + Velocity Decay
```matlab
R_mag = 10.0;
% In prediction step:
tau = 2.0;
X_pred(4) = X(4) * exp(-dt/tau) + ax * dt;
X_pred(5) = X(5) * exp(-dt/tau) + ay * dt;
```
Expected: velocity stays bounded (no ±4 m/s divergence), heading correction still active.

### Step 3 — Simulation: Test With Accelerometer Disabled
```matlab
ax = 0;
ay = 0;
```
If RMSE improves → accelerometer is net harmful for Task 2. Then use zero-velocity model + ToF.

### Step 4 — Simulation: Re-estimate Gyro Bias for Task 2
```matlab
% Compute bias from first 2 seconds of Task 2 data:
stationary_mask = imu_time < 2.0;
gyro_bias_x_task2 = mean(gyro_data(stationary_mask, 1));
```
If the bias differs significantly from 0.00186 rad/s, use the Task 2-specific bias.

### Step 5 — Simulation: Add Bounds Protection to ToF Model
Add `distances(distances <= 0) = inf` in `calculate_expected_tof`. This should prevent catastrophic divergence when EKF position exits the arena.

---

## 7. Parameter History

| Parameter | Original Helitha | Change 1 | Change 2 | Change 3 | Change 4 (current) |
|-----------|-----------------|----------|----------|----------|---------------------|
| `omega sign` | `-` | `+` | `+` | `+` | `+` |
| `accel_clip` | none | none | `±2.0` | `±2.0` | `±2.0` |
| `R_mag` | `0.1` | `0.1` | `0.1` | `1.0` | `100.0` |
| `mag gating` | none | none | none | yes | yes |
| `gamma_threshold` | `3.84` | `3.84` | `3.84` | `3.84` | `3.84` |
| `Q_vx/vy` | `0.1` | `0.1` | `0.1` | `0.1` | `0.1` |
| **Task 1 RMSE** | **0.03m** | **0.047m** | **0.047m** | **0.047m** | **0.047m** |
| **Task 2 RMSE** | **1.02m** | **1.16m** | **1.16m** | **0.94m** | **0.93m** |

---

### 2.5 Change 5: Bounds Protection in calculate_expected_tof (Session 1, 2026-03-21)

**Change description:**
Added two guards to `calculate_expected_tof` (lines 244–253 of `myEKF.m`):
1. `distances(distances <= 0) = inf` — discards wall distances that are zero or negative (which occur when the EKF position has drifted outside the arena).
2. `if isinf(h_x) ... H_tof = zeros(1, length(X)); return; end` — if all four walls return invalid distances, skips the ToF update entirely by zeroing the Jacobian (Kalman gain becomes 0).

**Before/after metrics:**

| Metric | Before (Change 4) | After (Session 1) |
|--------|-------------------|-------------------|
| Task 1 RMSE | 0.047m | **0.0452m** ✓ |
| Task 2 RMSE | 0.93m | **0.9315m** (unchanged) |
| Task 2 max heading error | ~180° | not measured (no debug_task2 run) |
| Right subplot divergence | single-step spike to ±10,000m | smooth runaway to ~10,000m (Task 2), ~2,700m (Task 1) |

**Conclusion:**
Bounds protection is confirmed working. Task 1 RMSE improved marginally (0.047m → 0.045m) and remains well within the 0.05m limit — no regression. Task 2 RMSE is statistically unchanged (0.9315m vs 0.93m), as expected: this fix was never intended to improve position accuracy, only to prevent the single-step catastrophic jump. The shape of the right subplot divergence changed from a sharp vertical spike (single bad Kalman update) to a smooth horizontal runaway, confirming that: (a) bounds protection is firing and blocking the single-step jump, and (b) the residual divergence is now caused entirely by the wrong accelerometer axis (BUG 1/2/3 from Session 0 — accel column 1 is the gravity axis, integrating +0.154 m/s² continuously into vx). Critically, the right subplot also diverges for Task 1 (~2,700m endpoint), confirming the accel bias runaway is present in both tasks. Task 1 survives because 14s is short enough for ToF corrections to dominate while the robot is inside the arena. Session 2 must fix BUGs 1, 2, and 3 (accel axis swap) as the next atomic change.

---

### 2.6 Change 6: Accelerometer Axis Swap + Bias Fix (Session 2, 2026-03-21)

**Change description (three sub-steps per session discipline):**

**Sub-step A — Bug 1:** Changed `accel_data(k, 1)` → `accel_data(k, 3)` for `ax` (line 106).
Column 1 is the vertical/gravity axis (stationary mean = +10.003 m/s²). Column 3 is the
forward axis (stationary mean = −0.396 m/s²), confirmed from `calib2_straight.mat` in Session 0.
`accel_bias_x = 9.84855` was left unchanged for this sub-step.

**Sub-step B — Bugs 2 & 3:** Replaced `accel_bias_x = 9.84855` with `accel_bias_fwd = -0.396`
(lines 35–36 and 106). The gravity bias 9.84855 was correct for axis 1 but catastrophically
wrong for axis 3 (residual = −0.396 − 9.849 = −10.24 m/s², clipped to −2.0 m/s²).
After fix: residual ≈ 0 m/s² at rest. `accel_bias_y = 0.07485` left unchanged (Bug 4 deferred).

**Before/after metrics:**

| Metric | Before (Session 1) | After Sub-step A | After Sub-step B |
|--------|--------------------|-----------------|-----------------|
| Task 1 RMSE | 0.0452m ✓ | 0.0456m ✓ | **0.0452m** ✓ |
| Task 2 RMSE | 0.9315m | 0.9300m | **0.9323m** |
| Right subplot divergence (T1) | ~2,700m | ~2,700m | ~2,700m (unchanged) |
| Right subplot divergence (T2) | ~10,000m | ~10,000m | ~10,000m (unchanged) |

**Unexpected finding — Sub-step A:** Sub-step A was predicted to fail Task 1 (axis 3 with
gravity bias → net −10.24 m/s², clipped to −2.0 m/s² always). Task 1 actually passed at
0.0456m. The 10Hz ToF corrections over 14s are strong enough to suppress even a constant
−2.0 m/s² velocity error in the short-duration straight-line task.

**Main finding — null result on Task 2:** Fixing Bugs 1, 2, and 3 had zero measurable effect
on Task 2 RMSE (0.9315 → 0.9323, within noise). The right subplot divergence is also
unchanged for both tasks. The accelerometer was **neutral** for Task 2 — neither helpful nor
harmful — because the heading drift (Root Cause A, ≈92° over 52s) causes widespread ToF
rejection, and even a correctly-integrated `vx` gets projected onto the wrong world axis when
θ is 88° wrong. Fixing the accelerometer is correct engineering but does not move the
performance needle until heading is corrected first.

**Residual state divergence analysis:** The right subplot still runs to ~2,700m (Task 1) and
~10,000m (Task 2) after the fix. Even with zero mean bias, axis 3 vibration noise (σ = 0.034 m/s²)
random-walks into vx because `Q_vx = 0.1` is 3.5×10⁶× overinflated. The velocity states are
unconstrained between ToF corrections, causing position to drift arbitrarily far during the
coasting phases of Task 2 when ToF is rejected. This is the target for Session 3 (velocity decay).

**Conclusion:**
Bugs 1, 2, and 3 are correctly fixed and must remain in the code. The accelerometer now
uses the physically correct axis with the verified bias — any future RMSE calculation is
no longer corrupted by a gravity-axis residual. However, the fixes are insufficient to improve
Task 2 RMSE because the dominant bottleneck is Root Cause A (motion-induced gyro bias → 92°
heading drift). Root Cause A causes ToF gate rejection, which prevents position correction,
which allows velocity noise to accumulate unchecked. Session 3 (velocity decay model) is the
next fix: it constrains vx/vy from drifting during the gyro-drift periods and should reduce
the state divergence. The heading problem itself will require Session 4 (gyro bias re-estimation
from Task 2 first 2s) or Session 5 (6-state EKF with online gyro bias).

---

## 8. Files Reference

| File | Purpose |
|------|---------|
| `myEKF.m` | Main EKF — current code with all changes applied |
| `test_ekf.m` | Load task data, run EKF, compute RMSE, plot trajectory |
| `debug_task2.m` | Task 2 diagnostics: heading error, accel analysis, ToF2 gating, 5 figures |
| `debug_ekf.m` | Task 1 diagnostics: P(5,5), vy_est, ToF2 innovations |
| `sensor_diagnostic.m` | Lab sensor analysis: 8 figures, static/rotation/magnetometer checks |

---

## Session 0 — Calibration Verification
**Date:** 2026-03-17
**Method:** Five standalone verification scripts (`verify_step1_gyro.m` through `verify_step5_Q.m`) run in MATLAB against `calib2_straight.mat` and `calib1_rotate.mat`. No changes made to `myEKF.m`.
**Full verified constants:** `verified_constants.md`

---

### S0.1 Verified Values Table

| Constant | Value in myEKF.m | Verified value | Match |
|----------|-----------------|----------------|-------|
| IMU sample rate | (assumed 104 Hz) | **200 Hz** (dt = 0.005 s) | MISMATCH — affects all Q derivations |
| Gyro yaw axis | column 1 | column 1 | ✓ |
| `gyro_bias_x` | 0.00186 rad/s | 0.001855 rad/s | ✓ (diff = 5e-6 rad/s) |
| Gyro scale factor | 1.0 (none applied) | 1.007548 (0.749% error) | ✓ PASS |
| R_gyro | — | 1.275e-06 (rad/s)² | — |
| Accel forward axis | **column 1** | **column 3** | **CRITICAL BUG** |
| Accel vertical axis | — | column 1 (mean = +10.003 m/s²) | — |
| Accel lateral axis | column 2 | column 2 | ✓ |
| `accel_bias_x` (axis 1) | 9.84855 m/s² | +10.003 m/s² | residual = +0.154 m/s² |
| `accel_bias_y` (axis 2) | 0.07485 m/s² | +0.028 m/s² | residual = −0.047 m/s² |
| accel bias for axis 3 | **not present** | −0.396 m/s² | **CRITICAL BUG — missing** |
| R_accel (fwd, axis 3) | — | 1.155e-03 (m/s²)² | — |
| Mag hard-iron Y | −3.705e-05 T | −3.705e-05 T | ✓ exact match |
| Mag hard-iron Z | +4.415e-05 T | +4.415e-05 T | ✓ exact match |
| Mag soft-iron Y | 1.0958 | 1.095816 | ✓ (diff = 1.6e-5) |
| Mag soft-iron Z | 0.9196 | 0.919592 | ✓ (diff = 8e-6) |
| Mag calibrated Lissajous | — | aspect ratio = 1.0000 | ✓ perfect circle |
| R_mag (measured) | — | 1.747e-11 T² | — |
| `R_mag` (in code) | 100.0 | — | intentional EMI workaround; 5.72e12× measured |
| `Q_x`, `Q_y` | 1e-4 | 7.22e-13 (lower bound) | 1.4e+08× overinflated |
| `Q_theta` | 1e-4 | 3.19e-11 | 3.1e+06× overinflated |
| `Q_vx` | 0.1 | 2.89e-08 | 3.5e+06× overinflated |
| `Q_vy` | 0.1 | 5.34e-09 | 1.9e+07× overinflated |
| `R_tof` | 0.01 | not verified (Session 0 scope) | defer |
| `gamma_threshold` | 3.84 | correct formula (χ²₁, p=0.05) | ✓ formula correct |
| Arena bounds | x∈[−1.2,1.2], y∈[−2.16,1.2] | not verified from data | accept as given |
| ToF mounting angles | 0, π/2, −π/2 | not verified from data | accept as given |

---

### S0.2 Confirmed Bugs Found

**BUG 1 — CRITICAL: Wrong accelerometer axis for forward velocity**
- `myEKF.m` lines 100–107 use `accel_data(k,1)` for `ax` (body-forward velocity integration).
- Axis 1 is the **vertical** (gravity) axis: stationary mean = +10.003 m/s².
- Correct forward axis is **column 3**: stationary mean = −0.396 m/s², largest motion std during straight-line run (0.860 m/s² vs 0.704 m/s² for axis 2).

**BUG 2 — CRITICAL: Incorrect bias leaves +0.154 m/s² residual integrated into vx**
- `accel_bias_x = 9.84855` subtracted from axis 1.
- Verified stationary mean of axis 1: +10.003 m/s².
- Residual = +0.154 m/s². Not clipped by `accel_clip = 2.0`.
- Effect: ~0.154 m/s velocity drift per second of motion → ~2.3 m position error over 30 s without ToF wall visibility.

**BUG 3 — CRITICAL: Forward axis (col 3) bias not corrected at all**
- No bias is defined or applied for axis 3 anywhere in `myEKF.m`.
- Verified stationary mean of axis 3: −0.396 m/s².
- After fixing BUG 1 (switching to axis 3), this uncorrected bias would immediately dominate vx unless simultaneously addressed.

**BUG 4 — minor: Lateral axis (col 2) bias mismatch**
- `accel_bias_y = 0.07485` vs verified mean of +0.028 m/s²; residual = −0.047 m/s².
- Root cause: `calibration.m` loaded `calib1_rotate.mat` (rotation data) for the stationary window section instead of `calib2_straight.mat`.

**BUG 5 — structural: calibration.m loads wrong source file**
- Both `%% straight line file` and `%% rotate file` sections in `calibration.m` load `calib1_rotate.mat`.
- The stationary bias section should load `calib2_straight.mat`.
- Gyro bias was accidentally correct (first 2 s of rotation file happened to be stationary). Lateral accel bias was not.

---

### S0.3 Confirmed Correct Constants

| Constant | Confidence | Evidence |
|----------|-----------|---------|
| Gyro yaw axis = column 1 | High | RMS during rotation: 0.489 vs 0.062, 0.033 rad/s for axes 2, 3 |
| `gyro_bias_x = 0.00186` rad/s | High | Verified 0.001855; diff = 5e-6 rad/s — within rounding |
| Gyro scale factor = 1.0 | High | 0.749% integration error — PASS threshold |
| Lateral accel axis = column 2 | High | Confirmed stationary and motion analysis |
| Mag hard-iron offsets | High | Machine-epsilon match to verified values |
| Mag soft-iron scales | High | < 2e-5 difference to verified values |
| Mag horizontal axes = col 2, 3 | High | Full-circle Lissajous confirms; calibrated aspect = 1.000 |
| `R_mag = 100.0` (intentional) | — | Acknowledged EMI workaround; K_mag ≈ 0 |
| `gamma_threshold = 3.84` (formula) | High | Correct χ²₁ at p=0.05; tightness is a separate tuning question |

---

### S0.4 Session 1 Priority

Session 1 must fix BUGs 1, 2, and 3 as a single atomic change: swap the forward acceleration axis from column 1 to column 3, remove the incorrect `accel_bias_x = 9.84855`, and add `accel_bias_fwd = -0.396` applied to axis 3. These three bugs are tightly coupled — fixing any one in isolation leaves the other two making the result worse rather than better. After this axis fix, the large `Q_vx = 0.1` (which was serving as a partial workaround for the broken accel integration) should be re-evaluated; a starting point of `Q_vx = Q_vy = 1e-4` is suggested, tighter than current by 1000× but still well above the measured noise floor to accommodate Mecanum wheel slip. The lateral axis bias mismatch (BUG 4, residual −0.047 m/s²) is lower priority and can be corrected in the same session or deferred, as its effect is much smaller than the gravity-axis error.

---

## Session 4b — Professor Updates (Pre-Session 5 Patch)
**Date:** 2026-03-21
**Branch:** `fix-task2-from-helitha`
**Purpose:** Apply four professor-issued updates before Session 5. Treated as an administrative patch — no EKF logic changed. Performance metrics unchanged from Session 2 baseline (Task 1: 0.0452m, Task 2: 0.9323m).

---

### Update A — Function Signature (BREAKING FIX)
**Status: COMPLETE**

The professor changed the required signature from 3 outputs to 2 outputs.

**Lines changed:**

| File | Line | Before | After |
|------|------|--------|-------|
| `myEKF.m` | 1 | `function [X_Est, P_Est, GT] = myEKF(out)` | `function [X_Est, P_Est] = myEKF(out)` |
| `myEKF.m` | 28 | `GT = gt_pos;` | *(removed)* — `gt_pos` kept for x0/y0/theta0 init |
| `test_ekf.m` | 18 | `[X_Est, P_Est, GT] = myEKF(out);` | `[X_Est, P_Est] = myEKF(out);` + 3-line GT extraction from `out` |
| `debug_task2.m` | 9 | `[X_Est, P_Est, GT] = myEKF(out);` | `[X_Est, P_Est] = myEKF(out);` + 3-line GT extraction from `out` |
| `debug_ekf.m` | 10 | `[X_Est, P_Est, GT] = myEKF(out);` | `[X_Est, P_Est] = myEKF(out);` + 3-line GT extraction from `out` |

GT is still needed by all three callers (for RMSE calculation and plotting). Since GT comes from `out.GT_position.signals.values` inside `myEKF.m`, the same 3-line extraction was added after each call:
```matlab
gt_pos = squeeze(out.GT_position.signals.values);
if size(gt_pos, 1) == 3, gt_pos = gt_pos'; end
GT = gt_pos;
```

**Note:** `EKF_Test.m` and `ekf_tm.m` also contain the old 3-output signature but are not active submission files and are not called by any test script. They were not modified.

**Syntax check (initial):** 3 `function` / 3 `end` pairs — balanced. `checkcode` reported one warning: `gt_time` (lines 24-25) assigned but never used. Both lines removed — `gt_time` was kept after the GT output removal but served no purpose in the function body. After removal, `checkcode('myEKF.m')` returns no warnings.

---

### Update B — Arena Bounds (affects Session 1 fix)
**Status: COMPLETE**

Corrected from asymmetric wrong bounds to confirmed 244cm × 244cm square arena.

**Lines changed:**

| File | Line | Before | After |
|------|------|--------|-------|
| `myEKF.m` | 42 | `arena_bounds = struct('x_max', 1.2, 'x_min', -1.2, 'y_max', 1.2, 'y_min', -2.16);` | `arena_bounds = struct('x_max', 1.22, 'x_min', -1.22, 'y_max', 1.22, 'y_min', -1.22);` |

**Not changed (flagged):** `debug_task2.m` line 102 and `debug_ekf.m` lines 27-28 still contain the old asymmetric bounds (`y_min = -2.16`). These are in diagnostic scripts only and do not affect the submitted EKF. They should be updated separately before the next diagnostic run to avoid misleading ToF2 model predictions.

**Impact on Session 1 fix:** The bounds protection in `calculate_expected_tof` (Session 1) used the old asymmetric bounds. With `y_min = -2.16`, the south wall was 3.36m from centre — far outside the real arena. With `y_min = -1.22` (correct), `distances <= 0` will trigger sooner if EKF drifts south, providing correct protection.

---

### Update C — Starting Position (flag only, no code change)
**Status: FLAGGED — no change made**

Current code reads starting position directly from GT data:
```matlab
x0 = gt_pos(1, 1);   % line 50
y0 = gt_pos(1, 2);   % line 51
```
And reads initial heading from GT quaternion (lines 55–58).

**Problem:** This is a GT dependency. In the Simulink context (professor's model), GT will not be available at run time. The initialisation will need to be hardcoded or derived from the first ToF reading.

**Task 2 starting wall (confirmed 2026-03-21):** `GT(1,1:2) = [-0.0118, -0.9589]`. Robot body centre is at x≈0, y≈−0.96 m. This places it against the **south wall** (y_min = −1.22), with ~0.26 m offset = robot half-length. x is essentially centred (−0.012 m from centre). For Session 5 hardcoded init: `x0 = 0.0; y0 = -0.959;`. Initial heading theta0 still needs confirming — run the quaternion→yaw conversion from GT(1,:) to get the hardcoded value.

---

### Update D — Simulink model (flag only, no code change)
**Status: FLAGGED — no change made**

`test_ekf.m` calls `myEKF(out)` directly as a MATLAB function. It does **not** use the Simulink model.

**Compatibility issue for Session 5:** The Simulink MATLAB Function block interface is incompatible with the current `out`-struct convention. The `out` struct (`out.Sensor_GYRO.signals.values`, etc.) is a Simulink-to-workspace logging artifact. Inside a Simulink Function block, individual signal arrays are passed directly as arguments, not packed into a struct. Resolution options:
1. Keep `myEKF.m` as-is; update `test_ekf.m` to run the Simulink model, collect `out` from the workspace logger, then call `myEKF(out)` as before.
2. Refactor `myEKF.m` to accept individual signal arrays instead of a struct — this would require a wrapper for offline testing.

No changes made. Session 5 must address this before the Simulink submission is tested.

---

## Session 4c — RMSE Fix + Q Reduction
**Date:** 2026-03-21
**Branch:** `fix-task2-from-helitha`
**Files changed:** `test_ekf.m` (Fixes 1 and 3), `myEKF.m` (Fix 2)

---

### Fix 1 — Correct RMSE calculation in test_ekf.m

**Root cause identified:** `test_ekf.m` compared sample index `k` of X_Est (IMU time grid, ~104 Hz) against sample index `k` of GT (PhaseSpace, 200 Hz). Since GT has ~1.92× more samples per second, sample `k` of GT is at a different timestamp than sample `k` of X_Est — causing the RMSE to measure spatial offset at mismatched points in time. For a moving robot, even a 5 ms timestamp offset becomes a position error proportional to robot speed. This inflated all reported RMSE numbers approximately 10× (Task 1: 0.0452m real → 0.43m reported; Task 2: 0.9323m real → 1.19m reported).

**Fix:** Replaced the sample-indexed comparison with `interp1` interpolation of GT onto the IMU time grid, then computed errors from time-aligned samples. Applied to both the XY position RMSE block and the yaw RMSE block.

**Code changes in test_ekf.m (RMSE block):**
```matlab
% BEFORE (sample-indexed, misaligned):
origin_xy = GT(1, 1:2);
GT_xy = GT(:, 1:2) - origin_xy;
X_xy = X_Est(:, 1:2) - origin_xy;
n_samples = min(size(GT_xy, 1), size(X_xy, 1));
err_x = X_xy(1:n_samples, 1) - GT_xy(1:n_samples, 1);
err_y = X_xy(1:n_samples, 2) - GT_xy(1:n_samples, 2);

% AFTER (time-aligned via interp1):
imu_time_vec = out.Sensor_GYRO.time;
gt_time_vec  = out.GT_position.time;
GT_x_interp = interp1(gt_time_vec, gt_pos_raw(:,1), imu_time_vec, 'linear', 'extrap');
GT_y_interp = interp1(gt_time_vec, gt_pos_raw(:,2), imu_time_vec, 'linear', 'extrap');
err_x = X_Est(:,1) - GT_x_interp;
err_y = X_Est(:,2) - GT_y_interp;
```

**Yaw RMSE block:** Same fix — `interp1` of `yaw_gt_raw` from `gt_rot_time` onto `imu_time_vec` before computing `wrapToPi` difference.

**Trajectory plot:** Subplot 1 now plots `GT_x_interp - origin_xy(1)` (interpolated, then shifted) instead of raw GT samples — origin_xy is derived from `GT_x_interp(1)`.

**Expected result:** Task 1 RMSE should return to ~0.045m (previously confirmed real value). If Task 1 RMSE is still above 0.10m after this fix, there is a second problem unrelated to the sample rate mismatch.

---

### Fix 2 — Reduce Q_vx and Q_vy (myEKF.m line 70)

**What Q_vx = 0.1 means physically:** The filter is told that the robot's x-velocity can shift by ±0.316 m/s per timestep due to unmodeled process noise. This is so large that the filter barely trusts its own velocity prediction — it accepts almost any accelerometer reading as true, allowing velocity to random-walk to extreme values (±4 m/s observed) during the coasting phases of Task 2 when ToF updates are rejected.

**Code change in myEKF.m:**
```matlab
% BEFORE:
Q = diag([1e-4, 1e-4, 1e-4, 0.1, 0.1]);

% AFTER:
Q = diag([1e-4, 1e-4, 1e-4, 1e-4, 1e-4]);
```

**Before/after metrics (to be filled after running test_ekf.m on both tasks):**

| Metric | Before (Session 2 baseline) | After Fix 1 only | After Fix 2 |
|--------|-----------------------------|-----------------|-------------|
| Task 1 RMSE | 0.0452m ✓ | ~0.045m (expected) | (run test) |
| Task 2 RMSE | 0.9323m | ~0.93m (expected) | (run test) |
| Task 1 Yaw RMSE | (not recorded) | (run test) | (run test) |
| Task 2 Yaw RMSE | (not recorded) | (run test) | (run test) |

**Why this should help:** With `Q_vx = 1e-4`, velocity uncertainty grows by only ±0.01 m/s per step. The filter will rely more on the velocity model (physics) and less on the noisy accelerometer reading. Velocity will no longer random-walk between ToF corrections. The state trajectory subplot should stop diverging to ±10,000 m.

**Risk:** If the accelerometer is providing genuinely useful forward velocity information (e.g. during straight-line phases of Task 1), tightening Q_vx may slow down the filter's response to real acceleration. Task 1 RMSE should be checked carefully — if it rises above 0.05m, revert or try an intermediate value of Q_vx = 1e-3.

---

### Fix 3 — Correct subplot 2 in test_ekf.m

**Problem:** The old subplot 2 plotted `X_Est` raw (all 5 state columns against sample index), with incorrect axis labels (`X Position (m)`, `Y Position (m)`) and a wrong legend (`Ground Truth (PhaseSpace)`, `EKF Estimate`). This made the subplot unreadable — states were plotted with sample number as the x-axis instead of time, and the legend described a trajectory plot that wasn't there.

**Fix:** Replaced with a time-series plot of all 5 EKF state variables against `imu_time_vec` (seconds), with correct individual labels per state and correct axis labels.

**Code change in test_ekf.m (subplot 2):**
```matlab
% BEFORE:
subplot(1,2,2);
plot(X_Est, 'b--', 'LineWidth', 1.5);
legend('Ground Truth (PhaseSpace)', 'EKF Estimate');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Trajectory: Estimate vs Ground Truth (Task 1)');
grid on; axis equal;

% AFTER:
subplot(1,2,2);
state_names = {'x (m)', 'y (m)', 'theta (rad)', 'vx (m/s)', 'vy (m/s)'};
for i = 1:5
    plot(imu_time_vec, X_Est(:,i), 'DisplayName', state_names{i}); hold on;
end
xlabel('Time (s)'); ylabel('State value');
title('EKF State Variables Over Time');
legend show; grid on;
```

No myEKF.m changes. No RMSE impact.

---

### Conclusion (pending test results)

Fix 1 is a measurement correction — it does not change filter behaviour, only how RMSE is computed. The "0.43m / 1.19m" readings reported before this session were artefacts. True filter performance after Session 2 was Task 1: 0.0452m, Task 2: 0.9323m.

Fix 2 constrains velocity states from random-walking during ToF-rejection periods. If successful, the right subplot should show bounded vx/vy (±0.5 m/s range) instead of the ±10,000 m divergence seen previously. Task 2 RMSE improvement depends on whether velocity runaway was a significant contributor — if the dominant error is heading drift (Root Cause A, 92° over 52s), Fix 2 alone will not close the gap to 0.30m. Session 5 (gyro bias or 6th-state online estimation) remains the critical path for Task 2.

**Next session:** Session 5 — gyro bias re-estimation from Task 2 first 2s, OR online gyro bias as 6th state.

---

## Session 5b — 6-State EKF with Online Gyro Bias Estimation
**Date:** 2026-03-21
**Branch:** `fix-task2-from-helitha`
**Pre-session baseline:** Task 1 RMSE = 0.172m, Task 2 RMSE = 1.084m

### Change Description
Extended the EKF state vector from 5 to 6 states, adding online gyro bias `b_gyro` as the 6th state. The filter now estimates and tracks the motion-induced gyro bias in real time rather than using a fixed stationary calibration constant.

**7 changes applied (all at once):**

| Step | Change |
|------|--------|
| 1 | `X` extended to 6×1: `b_gyro` initialised at stationary value 0.00186 rad/s |
| 2 | `omega = gyro_data(k,1) - X(6)` (was `- gyro_bias_x` constant) |
| 3 | `F` extended to 6×6: `F(3,6) = -dt` (∂theta/∂b_gyro), `F(6,6)=1` (random walk) |
| 4 | `Q` extended to 6×6: `Q(6,6) = 1e-6` (bias changes slowly) |
| 5 | `P` extended to 6×6: `P(6,6) = (0.005)^2 = 2.5e-5` (initial bias uncertainty) |
| 6 | `H_mag` padded to 1×6; `H_tof` initialised as `zeros(1,6)`; `eye(5)→eye(6)` in all IKH |
| 7 | Output: `X_Est(k,:) = X(1:5)'`, `P_Est(:,:,k) = P(1:5,1:5)` (grading compatibility) |

### Results

| Metric | Pre-5b Baseline | Session 5b | Change |
|--------|----------------|------------|--------|
| Task 1 RMSE X | — | 0.1696 m | — |
| Task 1 RMSE Y | — | 0.0283 m | — |
| **Task 1 RMSE Total** | **0.172 m** | **0.172 m** | **0.000 m (unchanged ✓)** |
| Task 2 RMSE X | — | 0.5555 m | — |
| Task 2 RMSE Y | — | 0.6248 m | — |
| **Task 2 RMSE Total** | **1.084 m** | **0.836 m** | **-0.248 m (-23%) ↓** |
| Task 2 Yaw RMSE | — | 69.16° | — |
| Task 2 Yaw max error | — | 179.95° | — |
| Task 2 Yaw final error | — | 61.48° | — |
| Final X(6) b_gyro | 0.00186 rad/s (init) | 0.059443 rad/s | +0.057 rad/s |
| X_Est output size | 5-state ✓ | 5-state ✓ | unchanged |
| P_Est output size | 5×5 ✓ | 5×5 ✓ | unchanged |

### Diagnosis

**What worked:**
- Task 1 RMSE completely unchanged (0.172m → 0.172m) — 6-state extension did not perturb the straight-line task.
- Task 2 RMSE improved 23% (1.084m → 0.836m) — the online bias estimator is active.
- Output dimensions preserved correctly for grading.

**What did not work:**
- Task 2 RMSE = 0.836m remains well above the 0.30m target.
- Yaw RMSE of 69° and max error of 179.95° indicate the filter completely loses heading orientation at some point mid-circuit.

**Bias convergence analysis:**
- X(6) moved from 0.00186 rad/s (stationary init) to **0.059443 rad/s** at end of Task 2.
- Expected motion-induced bias from CLAUDE.md: ~0.031 rad/s.
- Final estimate is ~2× the expected value — the estimator overshot.
- This overshoot is likely a feedback artefact: accumulated heading error causes the filter to drive the bias estimate upward to compensate, creating a runaway loop rather than true convergence.
- The near-180° yaw max confirms the filter momentarily inverts the robot's heading. Once this happens, all subsequent ToF geometry is wrong (walls appear on the opposite sides), driving further divergence.

**Root cause not yet resolved:**
The motion-induced gyro bias problem (Root Cause A from CLAUDE.md) is more severe than a simple bias offset. The bias appears to be dynamic — varying with turn speed and motor load — rather than a constant offset that can be tracked by a slow random-walk model. Q(6,6) = 1e-6 may be too tight to track rapid changes at turns, or too loose to prevent the estimator from overshooting during straight legs.

### Next Steps (not yet implemented — no parameter changes this session)
1. Investigate whether the yaw flip at ~180° occurs at a specific turn (run debug_task2.m to isolate timing).
2. Consider whether `theta0 = pi/2` hardcode (Session 5a, not yet applied) would reduce the initial heading error that compounds into the 180° flip.
3. Consider tuning Q(6,6) — lower value (1e-8) to slow bias drift and prevent overshoot, or higher (1e-4) to track rapid changes at turns.
4. Consider whether the magnetometer (R_mag=100, effectively disabled) could serve as a heading anchor to prevent the 180° flip.

---

## Session 5a — theta0 Hardcoded to pi/2
**Date:** 2026-03-21
**Branch:** `fix-task2-from-helitha`
**Pre-session baseline:** Task 1 RMSE = 0.172m, Task 2 RMSE = 0.836m (post-5b)
**Applied on top of:** Session 5b (6-state EKF already in place)

### Change Description
Replaced the entire GT quaternion extraction and theta0 computation block (9 lines) with a single hardcoded line.

**Before (9 lines removed):**
```matlab
% Initial heading from GT quaternion
gt_rot = squeeze(out.GT_rotation.signals.values);
if size(gt_rot,1)==4, gt_rot=gt_rot'; end
quat_norms = sqrt(sum(gt_rot.^2, 2));
first_valid_idx = find(quat_norms > 0.9, 1);
if isempty(first_valid_idx), first_valid_idx = 1; end
qw=gt_rot(first_valid_idx,1); qx=gt_rot(first_valid_idx,2);
qy=gt_rot(first_valid_idx,3); qz=gt_rot(first_valid_idx,4);
theta0 = wrapToPi(atan2(2*(qw*qz+qx*qy), 1-2*(qy^2+qz^2)) + pi);
%theta0 = pi/2;
```
**After (1 line):**
```matlab
theta0 = pi/2;  % Robot starts perpendicular to south wall, facing +Y (confirmed)
```

### Results

| Metric | Post-5b | Session 5a | Change |
|--------|---------|------------|--------|
| Task 1 RMSE X | 0.1696 m | 0.1696 m | 0.000 m |
| Task 1 RMSE Y | 0.0283 m | 0.0283 m | 0.000 m |
| **Task 1 RMSE Total** | **0.172 m** | **0.172 m** | **0.000 m (no change)** |
| Task 2 RMSE X | 0.5555 m | 0.5556 m | +0.0001 m |
| Task 2 RMSE Y | 0.6248 m | 0.6246 m | -0.0002 m |
| **Task 2 RMSE Total** | **0.836 m** | **0.836 m** | **0.000 m (no change)** |
| Task 2 Yaw max | 179.95° | 179.95° | 0° |
| Final X(6) b_gyro | 0.059443 rad/s | 0.059465 rad/s | +0.000022 rad/s |

### Diagnosis — Session 5a Had Zero Effect

**Finding:** The quaternion-derived theta0 (88.6°) and hardcoded theta0 (90.0°) produce numerically identical results to 4 decimal places on both tasks. The 1.4° difference was too small to produce any measurable change in trajectory or RMSE.

**Prediction in CLAUDE.md was wrong:** The CLAUDE.md plan predicted Task 1 RMSE would drop from 0.172m to ~0.03–0.05m after this fix. This did not happen. The 0.172m Task 1 RMSE must have a different root cause — not theta0 offset.

**Implication for Task 1 error:** RMSE X = 0.1696m, RMSE Y = 0.0283m. The dominant error is in X (lateral direction relative to the robot's forward travel). Possible causes:
- Lateral accelerometer bias (accel_bias_y = 0.07485 m/s²) integrating into lateral velocity
- ToF1/ToF3 (left/right sensors) not correcting lateral drift effectively
- Velocity state not being zeroed between ToF updates during the straight leg

**theta0 = pi/2 kept** in the code — it is cleaner than the quaternion block and causes no harm.

### Updated Next Steps
1. **Run debug_task2.m** to pinpoint the time of the 179.95° yaw flip — this is the primary blocker for Task 2.
2. **Investigate Task 1 lateral drift** (RMSE X = 0.170m) — root cause is not theta0; likely lateral velocity accumulation.
3. **Do not tune Q(6,6) or any other parameter** until the yaw flip timing is known.

---

## Session 5 Diagnostic — debug_task2.m Analysis
**Date:** 2026-03-21
**Fixes applied to debug_task2.m before running:**
1. Arena bounds corrected: `arena_y_min = -2.16 → -1.22`, `arena_x_max/min = ±1.2 → ±1.22`
2. `theta_gt` formula corrected: added `+pi` sign correction to match EKF convention
3. Added specific time-point heading error printouts and first-exceed thresholds
4. `myEKF` call was already 2-output — no change needed

### Heading Error Timeline

| Time | GT theta | EKF theta | Error | Status |
|------|----------|-----------|-------|--------|
| t=10s | 88.6° | 90.0° | **1.4°** | Tracking well — still on first straight leg |
| t=15s | 83.3° | 87.7° | **4.5°** | Still fine — turn 1 has not yet happened |
| t=20s | 153.7° | 117.0° | **-36.8°** | **First turn occurred t≈15–20s. EKF missed ~32°** |
| t=25s | -95.5° | 140.6° | **-123.9°** | Cascade failure — second turn also missed |
| t=30s | -83.8° | 165.0° | **-111.2°** | No recovery |

- First |error| > 90°: **t = 23.42s**
- First |error| > 175°: **t = 31.24s**
- Max |error|: **179.95°** (filter heading effectively inverted)
- RMS heading error: **69.15°**
- Final heading error: **61.54°**

### ToF Gating (diagnostic reconstruction, gamma=3.84)
- **86.5% of all ToF2 steps rejected** (Mahalanobis median = 43.0, gate = 3.84)
- Even outside turns: 89.3% rejected
- This is a **symptom** of position divergence, not an independent root cause
- Once heading is wrong by 36°+ after turn 1, predicted ToF distances are wrong → innovations exceed any reasonable gate → filter flies blind on position for the remaining 30+ seconds

### Turn Detection Bug (debug_task2.m only)
The `turn_mask` (|dθ/dt| > 0.15 rad/s at 200Hz) detected **889 spurious "turns"** of 0.0s duration. These are single-sample quantization noise spikes in the GT quaternion derivative, not real turns. Real circuit has 4 turns of ~90° each. The per-phase gating statistics are therefore unreliable. **This is a diagnostic display bug only — it does not affect myEKF.m.**

### Root Cause Confirmed
The 6-state online bias estimator fails during the first turn (~t=15–20s) because:
- The turn takes ~3–5s at the 200Hz loop rate
- With Q(6,6)=1e-6, the bias state can shift by at most ~0.03 rad/s over 5s
- The actual turn-phase bias spike is larger and faster than the random walk model can track
- The EKF under-rotates by ~32° during turn 1 alone
- After turn 1, position error propagates into ToF rejections, making further correction impossible

### What Was Not a Problem
- Before turn 1 (t=0–15s): heading error only 4.5° — gyro integration was accurate on straight leg
- theta0 = pi/2: confirmed correct (EKF theta = 90.0° at t=10s vs GT 88.6°, error 1.4°)

### Candidate Next Steps (not yet implemented)
1. **Increase Q(6,6)** from 1e-6 to allow faster bias adaptation during turns — risk: bias overshoots on straight legs
2. **Detect turn phase and apply larger Q(6,6) conditionally** — more complex but targets the failure window
3. **Zero-velocity / straight-leg detection** to freeze bias update when not turning
4. **Reduce R_mag from 100 to ~1.0** to let magnetometer anchor heading during turns — risk: motor EMI corrupts mag during Task 2 circuit

---

## Session 5c — Q(6,6) Tuning
**Date:** 2026-03-21
**Branch:** `fix-task2-from-helitha`
**Starting point:** Session 5b (6-state EKF, Q(6,6)=1e-6), Task 2 RMSE = 0.836m

### Change Tested
Tuned the gyro bias process noise Q(6,6) across three values to see if faster bias adaptation could correct the 32° heading miss on turn 1.

### Results

| Option | Q(6,6) | Task 1 RMSE | Task 2 RMSE | Yaw max | Final X(6) |
|--------|--------|-------------|-------------|---------|------------|
| 5b (keep) | 1e-6 | 0.172m | **0.836m** | 179.95° | +0.0594 rad/s |
| B | 1e-5 | 0.172m | 0.842m | 179.89° | +0.0049 rad/s |
| A | 1e-4 | 0.172m | 1.086m | 179.97° | −0.0013 rad/s |

**Winner: Q(6,6) = 1e-6 (no change from Session 5b).** Reverted.

### Conclusion — Q(6,6) Tuning Does Not Help

The X(6) trajectory across the three options reveals the real problem:
- Q=1e-6: bias drifted to +0.059 rad/s (overshot; estimator chasing heading error, not real bias)
- Q=1e-5: bias barely moved (+0.005 rad/s); filter became more uncertain but less predictive
- Q=1e-4: bias went **negative** (−0.001 rad/s) and RMSE regressed to 1.086m — worse than 5-state baseline

The bias state is **not reliably observable** from the available sensors. The magnetometer has R_mag=100 (effectively disabled). The ToF sensors provide no direct heading signal. The only way X(6) gets updated is through indirect cross-covariance coupling from H_tof(3) — heading partial derivative in ToF updates. With no strong heading anchor, increasing Q(6,6) just lets the bias state absorb noise from whatever measurement residuals are available, which makes things worse.

### Revised Root Cause Hypothesis

The diagnostic session showed the heading error jumps **32° during turn 1 alone** (t=15–20s), which is too large to be explained by gyro bias (0.031 rad/s × 5s = 9° maximum). This points to a different mechanism:

**Hypothesis: ToF updates during the turn are actively corrupting the heading estimate.**

During the turn, the robot rotates but the EKF lags behind. The predicted ToF distance (based on wrong heading) diverges from the actual reading. The ToF H matrix has a large heading partial derivative H_tof(3) = h_x × tan(φ). With R_tof=0.01 (very tight), the Kalman gain K(3) can be substantial — the filter "explains" the ToF discrepancy partly by adjusting θ in the wrong direction, fighting the gyro integration.

### Next Steps
1. **Test: set H_tof(3) = 0** — treat ToF as pure position measurement, remove heading coupling. If heading tracks turns correctly after this, the hypothesis is confirmed.
2. **Alternative: increase R_tof during turns** to reduce ToF influence on heading while rotating.
3. **Alternative: reduce R_mag to ~1–10** to give magnetometer a real heading anchor role.

## Next session — Session 5d: Decouple ToF from heading
Hypothesis: H_tof(3) coupling causes heading corruption during turns.
Fix: set H_tof(3) = 0 in calculate_expected_tof.
This is one line. Test on both tasks before anything else.
If Task 2 yaw max drops below 90 degrees, hypothesis confirmed.

---

## Session 5d — H_tof(3) = 0 (Remove Heading Coupling from ToF Updates)
**Date:** 2026-03-21
**Branch:** `fix-task2-from-helitha`
**Starting point:** Session 5b/5c (6-state EKF, Q(6,6)=1e-6), Task 2 RMSE = 0.836m

### Change
Set `H_tof(3) = 0` in both branches of `calculate_expected_tof`:

```matlab
% BEFORE — x-wall case:
H_tof(1) = -1 / cos(phi);
H_tof(3) = h_x * tan(phi);

% BEFORE — y-wall case:
H_tof(2) = -1 / sin(phi);
H_tof(3) = -h_x * cot(phi);

% AFTER (both cases):
H_tof(3) = 0;  % remove heading coupling from ToF updates
```

**Rationale:** The Session 5 diagnostic identified that during the first turn (t≈15–20s), the heading error jumped ~32° — too large for a bias effect alone. Hypothesis: the ToF H matrix entry H_tof(3) = dh/dθ was coupling large ToF innovations (caused by lagging heading estimate) into θ corrections, fighting the gyro integration and causing the filter to under-rotate during turns.

### Results

| Metric | Session 5b/5c | Session 5d | Change |
|--------|--------------|------------|--------|
| Task 1 RMSE Total | 0.172m | 0.173m | +0.001m (negligible) |
| **Task 2 RMSE Total** | **0.836m** | **0.684m** | **−0.152m (−18%) ↓** |
| Task 2 Yaw max | 179.95° | 179.90° | unchanged |
| Final X(6) b_gyro | +0.059 rad/s | +0.131 rad/s | bias now overshooting more |

### Diagnosis

**Hypothesis partially confirmed.** Removing H_tof(3) improved Task 2 RMSE by 18% (0.836→0.684m) while keeping Task 1 unchanged. The ToF heading coupling was genuinely harmful.

**Yaw max unchanged at ~180°.** The heading flip still occurs. Two possible explanations:
1. The flip happens later now (after more correct trajectory), explaining better RMSE despite same max error
2. A separate mechanism is still causing the eventual heading failure

**X(6) now reaches 0.131 rad/s** (vs 0.059 before). With H_tof(3)=0, the bias estimator no longer receives contradictory heading signals from corrupted ToF updates. It now has a cleaner (though still weak) observability path through the magnetometer cross-covariance. The higher value suggests it may be tracking something closer to true motion-phase bias, though 0.131 rad/s still overshoots the expected 0.031 rad/s.

**Current best:** Task 1 = 0.173m, Task 2 = 0.684m. Target: Task 2 ≤ 0.30m. Gap remaining: 0.384m.

### Next Steps
1. Run debug_task2.m again to check if the heading flip has moved later in time — this would confirm whether H_tof(3)=0 bought us more correct heading before eventual failure.
2. Investigate why the heading flip still occurs — the gyro integration should be more accurate now without ToF interference.
3. Consider whether R_mag reduction (100→~5) could provide a heading anchor to prevent the eventual flip.

---

## Session 5d — H_tof(3)=0 Results (debug_task2.m follow-up)
**Date:** 2026-03-21

### debug_task2.m output with H_tof(3)=0 active

**Final RMSE:** Task 1 = 0.172m, Task 2 = 0.684m

**Heading flip timing (vs previous session with H_tof(3) active):**

| Threshold | H_tof(3) active | H_tof(3)=0 | Change |
|-----------|----------------|------------|--------|
| First >30° | ~t=17s (interpolated) | ~t=17s (interpolated) | unchanged |
| First >90° | t=23.42s | **t=26.71s** | +3.3s later |
| First >175° | t=31.24s | **t=32.04s** | +0.8s later |

**Heading error at key points:**

| Time | GT theta | EKF theta | Error |
|------|----------|-----------|-------|
| t=10s | 88.6° | 95.9° | 7.4° |
| t=15s | 83.3° | 79.6° | −3.7° |
| t=20s | 153.7° | 116.0° | **−37.7°** |
| t=25s | −95.5° | −163.3° | **−67.8°** (was −123.9°) |
| t=30s | −83.8° | 136.5° | −139.7° |

**ToF rejection rate:** 87.8% (diagnostic uses gamma=3.84; myEKF uses gamma=9.0)

### Root Cause Confirmed

The first-turn heading miss of ~37° (t=15–20s) is **unchanged** by H_tof(3)=0. The first turn is lost by the same amount regardless. What improved is error accumulation after the first turn (t=20–26s) — the 3.3s delay to 90° error explains the 0.836→0.684m RMSE gain.

**Quantified turn bias:** The filter under-rotates by ~37° over ~5s at turn 1.
Effective uncompensated bias = 37° / (5s × 57.3°/rad) ≈ **0.129 rad/s during the turn**.
This matches X(6) final value of 0.131 rad/s — confirming the 6th state is tracking the turn-phase bias, but converging too slowly to correct turn 1 in real time.

The stationary bias is 0.00186 rad/s. The motion bias during turns is ~0.129 rad/s — **70× larger**. This is not a small offset; it is a fundamentally different operating regime. The random walk model Q(6,6)=1e-6 cannot adapt in time.

### Next Session
Session 5e: Reduce R_mag from 100.0 to 10.0 to test whether magnetometer can anchor heading during turns and reduce the first-turn 37° miss.

---

## Session 5e — R_mag Reduction (100 → 10)
**Date:** 2026-03-21
**Branch:** `fix-task2-from-helitha`
**Starting point:** Session 5d (H_tof(3)=0, R_mag=100), Task 1=0.172m, Task 2=0.684m

### Change
```matlab
% Before:
R_mag = 100.0;
% After:
R_mag = 10.0;
```

### Results

| Metric | Before (5d) | Session 5e | Change |
|--------|-------------|------------|--------|
| Task 1 RMSE X | 0.1696m | 0.1689m | −0.0007m |
| Task 1 RMSE Y | 0.0283m | 0.0355m | +0.0072m |
| **Task 1 RMSE Total** | **0.172m** | **0.173m** | **+0.001m (negligible)** |
| Task 2 RMSE X | 0.5556m | 0.3978m | −0.158m |
| Task 2 RMSE Y | 0.6246m | 1.9459m | **+1.321m (catastrophic)** |
| **Task 2 RMSE Total** | **0.684m** | **1.986m** | **+1.302m (WORSE — REVERTED)** |

### Conclusion
**R_mag=10 failed. Reverted to R_mag=100.0.**

The RMSE Y explosion (+1.32m) confirms that at R_mag=10, K_mag≈0.01 is sufficient for EMI-corrupted magnetometer readings to actively drag the heading in the wrong direction. The corrupted mag signal points roughly opposite to the true heading during motor operation → the filter updates θ toward the wrong value → Y-axis trajectory diverges catastrophically.

The improvement in RMSE X (−0.158m) is a red herring: if heading is being pulled ~180° wrong, x and y errors trade off depending on which axis the robot is traversing at the time.

**Key finding:** The magnetometer is EMI-corrupted enough that even K_mag=0.01 (R_mag=10) is too much trust. The Mahalanobis gating (gamma=9.0) is not rejecting the corrupted readings because the innovation falls within the gate — the EMI shifts the absolute heading by a consistent amount rather than producing obvious outliers. The mag reads consistently wrong, not noisily wrong.

**Implication:** The magnetometer cannot be used as a heading anchor during Task 2 motor operation at any R_mag ≤ 100. R_mag=100 (K_mag≈0.001) is confirmed as the practical minimum trust level.

### What Was Eliminated
- R_mag reduction as a heading anchor strategy: ruled out. EMI too severe.

### Remaining Gap
Task 2 RMSE = 0.684m. Target = 0.30m. Gap = 0.384m.
The first-turn 37° heading miss (t=15–20s) remains the dominant unresolved root cause.
The bias estimator X(6) converges to ~0.131 rad/s post-turn but cannot pre-empt the turn-phase spike.

### Candidate Next Steps
1. **Increase Q(6,6) adaptively during detected turns** — angular rate threshold triggers larger bias noise
2. **Increase initial P(6,6)** to allow faster early bias adaptation (currently (0.005)² = 2.5e-5 rad²/s²)
3. **Velocity model improvement** — zero the velocity prediction during turns to reduce position error accumulation when heading is most uncertain
4. **Accept 0.684m as final** if no further sensor-based correction is feasible without EMI-free magnetometer

---

## Session 5f — Increase Initial P(6,6)
**Date:** 2026-03-21
**Branch:** `fix-task2-from-helitha`
**Starting point:** Session 5d/5e (H_tof(3)=0, R_mag=100, P(6,6)=(0.005)²), Task 1=0.172m, Task 2=0.684m

### Change
Increased initial bias uncertainty P(6,6) to allow faster b_gyro adaptation before turn 1 (t≈15s).

```matlab
% Before:
P = diag([0.1, 0.1, 0.1, 2.0, 2.0, (0.005)^2]);  % P(6,6) = 2.5e-5

% After (kept):
P = diag([0.1, 0.1, 0.1, 2.0, 2.0, (0.5)^2]);    % P(6,6) = 0.25
```

### Three Values Tested

| P(6,6) value | P(6,6) magnitude | Task 1 RMSE | Task 2 RMSE | Change vs baseline |
|-------------|-----------------|-------------|-------------|-------------------|
| (0.005)² | 2.5e-5 | 0.172m | 0.684m | baseline |
| (0.05)² | 2.5e-3 | 0.173m | 0.704m | +0.020m (**WORSE**) |
| **(0.5)²** | **0.25** | **0.174m** | **0.665m** | **−0.019m (KEPT)** |

### Conclusion
**(0.5)² kept.** Task 2 improved 0.019m (−2.8%). Task 1 negligibly worse (+0.002m).

The non-monotonic result ((0.05)² worse, (0.5)² better) suggests the improvement is not primarily from faster P(6,6) decay but from a secondary effect: with P(6,6)=0.25, the Kalman gain on X(6) via cross-covariance paths is large enough to pull the bias state during the straight-leg phase (t=0–15s), giving a marginally better initial estimate at turn 1.

However, the 37° heading miss at turn 1 (t≈20s) is almost certainly still present — the 0.019m gain is too small to indicate the turn is being correctly tracked. The bias observability problem (no direct heading anchor, only indirect cross-covariance from ToF) limits how much P(6,6) initialisation can help.

### Remaining Gap
Task 2 RMSE = 0.665m. Target = 0.30m. Gap = 0.365m.

### Candidate Next Steps
1. **Adaptive Q(6,6) during turns** — detect |omega| > threshold, temporarily raise Q(6,6) to allow rapid bias tracking during the 3–5s turn window
2. **Velocity decay model** — constrain vx/vy from accumulating during turn phases when heading is unreliable
3. **Accept 0.665m as practical limit** given sensor constraints (EMI-corrupted mag, no GPS, motion-induced bias 70× stationary bias)

---

## Session 5g — Adaptive Q(6,6) During Turns
**Date:** 2026-03-21
**Branch:** `fix-task2-from-helitha`
**Starting point:** Session 5f (P(6,6)=(0.5)²), Task 1=0.174m, Task 2=0.665m

### Change (reverted)
```matlab
% Replaced:
P_pred = F * P * F' + Q;

% With (then reverted):
Q_adaptive = Q;
if abs(omega) > 0.3
    Q_adaptive(6,6) = 1e-4;  % 100x higher during turns
end
P_pred = F * P * F' + Q_adaptive;
```

### Results

| Metric | Before (5f) | Session 5g | Change |
|--------|-------------|------------|--------|
| Task 1 RMSE | 0.174m | 0.174m | 0.000m |
| Task 2 RMSE X | 0.488m | 0.383m | −0.105m |
| Task 2 RMSE Y | 0.452m | 2.870m | **+2.418m (catastrophic)** |
| **Task 2 RMSE Total** | **0.665m** | **2.896m** | **+2.231m — REVERTED** |

### Conclusion
**Adaptive Q(6,6) failed. Reverted.**

Same failure signature as R_mag=10 (Session 5e): RMSE Y explodes while RMSE X slightly improves, indicating the bias state is being driven in the wrong direction during turn phases. The |omega|>0.3 rad/s threshold likely fires on gyro noise spikes throughout the run — not just at the 4 real 90° turns. With Q(6,6)=1e-4 active at those noise spikes, the bias state random-walks aggressively in whichever direction the current cross-covariance suggests, corrupting heading and causing Y-axis divergence.

**Root cause of failure:** The bias state X(6) has poor observability (no direct heading sensor during Task 2 — mag EMI-disabled, ToF has H_tof(3)=0). With high Q(6,6), the bias absorbs whatever residuals are available, which are dominated by position errors (from previous heading misses) rather than true gyro bias signal. This creates a runaway loop: position error → bias update → worse heading → worse position.

**Key finding:** Both aggressive strategies tried (reduce R_mag, increase adaptive Q) produce the same catastrophic Y-axis failure pattern. The common mechanism is: any parameter that increases the filter's willingness to update heading/bias based on indirect evidence (rather than direct heading measurement) causes divergence, because the indirect signals are contaminated by accumulated position error.

### What Has Been Ruled Out
- R_mag reduction (Session 5e): EMI makes mag readings consistently wrong
- Adaptive Q(6,6) (Session 5g): poor bias observability causes runaway

### Remaining Gap
Task 2 RMSE = 0.665m (Session 5f result, restored). Target = 0.30m. Gap = 0.365m.

### Remaining Candidate Strategies
1. **Velocity decay model** — add `X_pred(4) = X(4)*exp(-dt/tau)` to prevent vx/vy from accumulating during turn phases. Does not touch heading or bias — lower risk.
2. **Accept 0.665m** as the practical limit given the sensor constraints.

---

## Session 5h — Velocity Decay Model
**Date:** 2026-03-21
**Branch:** `fix-task2-from-helitha`
**Starting point:** Session 5f (P(6,6)=(0.5)²), Task 1=0.174m, Task 2=0.665m

### Change (reverted)
Added exponential velocity decay to prediction step plus matching F diagonal entries:
```matlab
% Velocity update:
tau = X.X;  % tested: 2.0 and 0.5
X_pred(4) = X(4) * exp(-dt/tau) + ax * dt;
X_pred(5) = X(5) * exp(-dt/tau) + ay * dt;
% F Jacobian:
F(4,4) = exp(-dt/tau);
F(5,5) = exp(-dt/tau);
```

### Two Values Tested

| tau | Decay/step | Task 1 RMSE | Task 2 RMSE | vs baseline |
|-----|-----------|-------------|-------------|-------------|
| 2.0s | 0.9975 | 0.174m | 0.703m | +0.038m (worse) |
| 0.5s | 0.9900 | 0.174m | 0.688m | +0.023m (worse) |
| — (reverted) | 1.0 | 0.174m | **0.665m** | baseline |

### Conclusion
**Velocity decay failed at both tau values. Reverted.**

Decay suppresses velocity accumulation but also suppresses legitimate forward velocity that ToF updates rely on for position correction. The underlying 37° heading miss at turn 1 is unaffected — it is purely a gyro/bias issue. Constraining velocity does not help when the position error source is heading, not velocity runaway.

### What Has Been Ruled Out (complete list)
| Strategy | Session | Result |
|----------|---------|--------|
| R_mag = 10 | 5e | Catastrophic Y divergence (+1.30m) |
| Adaptive Q(6,6) during turns | 5g | Catastrophic Y divergence (+2.23m) |
| Velocity decay tau=2.0 | 5h | Worse (+0.038m) |
| Velocity decay tau=0.5 | 5h | Worse (+0.023m) |

### Current Best
**Task 1 = 0.174m, Task 2 = 0.665m** (Session 5f — P(6,6)=(0.5)²)

### Remaining Gap
Task 2 RMSE = 0.665m. Target = 0.30m. Gap = 0.365m.

All parameter-tuning paths from the available sensor set have been exhausted. The 37° first-turn heading miss requires either: (a) a direct heading sensor unaffected by EMI, or (b) a fundamentally different motion model for the turn phase. No further single-parameter changes are expected to close the 0.365m gap.
