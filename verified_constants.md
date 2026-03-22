# Verified Calibration Constants — Session 0
**Date:** 2026-03-17
**Branch:** `fix-task2-from-helitha`
**Source data:** `Training Data/calib2_straight.mat` (stationary/straight), `Training Data/calib1_rotate.mat` (5-rotation)
**Status:** Reference only — do NOT paste into `myEKF.m` until Session 1.

---

## Verified MATLAB Comment Block

Ready to replace the `%% Calibration Constants` section of `myEKF.m` in Session 1:

```matlab
% =========================================================
% VERIFIED CALIBRATION CONSTANTS — Session 0, 2026-03-17
% =========================================================
%
% Sample rate (verified from both recordings): 200 Hz (dt = 0.005 s)
% Session doc assumed 104 Hz — all Q derivations below use 200 Hz.
%
% Gyro
%   Yaw axis:              column 1
%   Stationary bias:       0.001855 rad/s  (from 12356 samples, 61.78 s)
%   myEKF.m value:         0.00186 rad/s   (diff = 5e-6 rad/s — OK)
%   Scale factor error:    0.749% vs 5-rotation ground truth — PASS, no correction needed
%   Scale factor value:    1.007548  (not applied in myEKF.m — acceptable)
%   Noise sigma:           0.001129 rad/s
%   R_gyro (verified):     1.275e-06  (rad/s)^2
%
% Accelerometer
%   Vertical axis:         column 1  (stationary mean = +10.003 m/s^2 — GRAVITY)
%                          *** DO NOT USE FOR VELOCITY INTEGRATION ***
%   Forward axis:          column 3  (stationary mean = -0.396 m/s^2)
%   Lateral axis:          column 2  (stationary mean = +0.028 m/s^2)
%   Noise sigma (fwd):     0.034 m/s^2
%   R_accel (verified):    1.155e-03  (m/s^2)^2
%
% Magnetometer
%   Horizontal axes:       Y = column 2, Z = column 3  (confirmed)
%   Hard-iron offset Y:   -3.705e-05 T   (EXACT MATCH to myEKF.m — OK)
%   Hard-iron offset Z:   +4.415e-05 T   (EXACT MATCH to myEKF.m — OK)
%   Soft-iron scale (Y/Z ratio raw): 0.839  (CORRECTION NEEDED — applied in myEKF.m)
%   Soft-iron scale Y:     1.095816       (EXACT MATCH to myEKF.m — OK)
%   Soft-iron scale Z:     0.919592       (EXACT MATCH to myEKF.m — OK)
%   Calibrated Lissajous:  aspect ratio = 1.0000 — perfect circle
%   Noise sigma Y:         3.199e-06 T
%   Noise sigma Z:         4.971e-06 T
%   R_mag (measured):      1.747e-11 T^2
%   R_mag (current tuned): 100.0   (NOT a noise measurement — EMI workaround,
%                                   inflated 5.72e12 x. K_mag ≈ 0. Root fix
%                                   is EMI rejection, not inflating R_mag.)
%
% Process noise Q (derived from verified sensor noise, dt = 0.005 s):
%   Q_x = Q_y:   7.22e-13  m^2       (current: 1e-4  — ratio: 1.4e+08 x)
%   Q_theta:     3.19e-11  rad^2      (current: 1e-4  — ratio: 3.1e+06 x)
%   Q_vx:        2.89e-08  (m/s)^2   (current: 0.1   — ratio: 3.5e+06 x)
%   Q_vy:        5.34e-09  (m/s)^2   (current: 0.1   — ratio: 1.9e+07 x)
% =========================================================
```

---

## Verified Values — Full Detail

### Gyro (Step 1)

| Parameter | myEKF.m value | Verified value | Source |
|-----------|--------------|----------------|--------|
| Yaw axis | column 1 | column 1 (RMS during rotation: 0.489 rad/s vs 0.062, 0.033 for axes 2,3) | `calib1_rotate.mat` |
| Stationary bias | 0.00186 rad/s | 0.001855 rad/s | `calib2_straight.mat`, 12356-sample window |
| Noise sigma | — | 0.001129 rad/s | same window |
| R_gyro | — | 1.275e-06 (rad/s)² | sigma² |
| Scale factor error | none applied | 0.749% (1.007548 correction) | 5-rotation integration |

### Accelerometer (Step 2)

| Parameter | myEKF.m value | Verified value | Source |
|-----------|--------------|----------------|--------|
| Vertical axis | — | column 1 (mean = +10.003 m/s²) | `calib2_straight.mat` stationary window |
| Forward axis | **column 1** ← BUG | **column 3** (mean = −0.396 m/s², largest motion std) | straight moving phase |
| Lateral axis | column 2 | column 2 (mean = +0.028 m/s²) | same |
| accel_bias_x (axis 1) | 9.84855 m/s² | +10.003 m/s² | residual = +0.154 m/s² |
| accel_bias_y (axis 2) | 0.07485 m/s² | +0.028 m/s² | residual = −0.047 m/s² |
| accel_bias for axis 3 | **not in code** | −0.396 m/s² | missing entirely |
| Forward axis noise sigma | — | 0.034 m/s² | axis 3 stationary std |
| R_accel (fwd) | — | 1.155e-03 (m/s²)² | sigma² |

### Magnetometer (Step 4)

| Parameter | myEKF.m value | Verified value | Match |
|-----------|--------------|----------------|-------|
| Horizontal axes | Y=col2, Z=col3 | Y=col2, Z=col3 | ✓ |
| Hard-iron Y | −3.705e-05 T | −3.705e-05 T | ✓ exact |
| Hard-iron Z | +4.415e-05 T | +4.415e-05 T | ✓ exact |
| Soft-iron Y | 1.0958 | 1.095816 | ✓ (diff 1.6e-5) |
| Soft-iron Z | 0.9196 | 0.919592 | ✓ (diff 8e-6) |
| Calibrated aspect ratio | — | 1.0000 | ✓ perfect circle |
| R_mag (noise-based) | — | 1.747e-11 T² | — |
| R_mag (in code) | 100.0 | — | intentional tuning, 5.72e12× measured |

### Process Noise Q (Step 5)

| Entry | Current | Derived (dt=0.005 s) | Ratio | Notes |
|-------|---------|----------------------|-------|-------|
| Q_x | 1e-4 | 7.22e-13 | 1.4e+08× | position |
| Q_y | 1e-4 | 7.22e-13 | 1.4e+08× | position |
| Q_theta | 1e-4 | 3.19e-11 | 3.1e+06× | from R_gyro × dt² |
| Q_vx | 0.1 | 2.89e-08 | 3.5e+06× | from R_accel_fwd × dt² |
| Q_vy | 0.1 | 5.34e-09 | 1.9e+07× | from R_accel_lat × dt² |

---

## Confirmed Bugs

### BUG 1 — CRITICAL: Wrong accelerometer axis used for forward velocity
- **Location:** `myEKF.m` lines 100–107
- **Code:** `ax = ... accel_data(k, 1) - accel_bias_x`
- **Problem:** Axis 1 is the vertical (gravity) axis — mean = +10.003 m/s². It does not measure forward acceleration.
- **Correct axis:** Column 3 (mean = −0.396 m/s², largest motion std during straight-line run)
- **Fix in Session 1:** Change `accel_data(k, 1)` → `accel_data(k, 3)` for `ax`; change bias accordingly.

### BUG 2 — CRITICAL: Incorrect bias applied to vertical axis, residual integrated into vx
- **Code:** `accel_bias_x = 9.84855` applied to axis 1
- **Verified mean of axis 1:** +10.003 m/s²
- **Residual:** +0.154 m/s² continuously added to vx every step (not clipped by accel_clip=2.0)
- **Effect:** ~0.154 m/s of velocity drift per second of motion; ~2.3 m position error over 30 s when walls are not visible to ToF sensors.

### BUG 3 — CRITICAL: Forward axis (col 3) bias entirely missing
- **Code:** No `accel_bias` defined or applied for axis 3
- **Verified mean of axis 3:** −0.396 m/s²
- **Effect:** Even after fixing BUG 1, the forward axis has an uncorrected −0.396 m/s² bias that will integrate into vx.
- **Fix in Session 1:** Add `accel_bias_fwd = -0.396; az = accel_data(k,3) - accel_bias_fwd;`

### BUG 4 — minor: Lateral axis (col 2) bias mismatch
- **Code:** `accel_bias_y = 0.07485`
- **Verified mean of axis 2:** +0.028 m/s²
- **Residual:** −0.047 m/s² (small; tolerable but incorrect)
- **Root cause:** `calibration.m` loaded `calib1_rotate.mat` (rotation data) instead of `calib2_straight.mat` for the stationary window, giving a wrong baseline for the lateral axis bias.

### BUG 5 — structural: calibration.m loads wrong file for stationary section
- **Code:** Both `%% straight line file` and `%% rotate file` sections load `calib1_rotate.mat`
- **Correct file for stationary bias:** `calib2_straight.mat`
- **Effect:** Gyro bias happened to be accurate (first 2 s of rotation file was stationary). Lateral accel bias was not.

---

## Confirmed Correct Constants

| Constant | Value | Confidence |
|----------|-------|-----------|
| Gyro yaw axis | column 1 | High — RMS 0.489 rad/s vs <0.063 for other axes |
| Gyro stationary bias | 0.00186 rad/s | High — diff from verified = 5e-6 rad/s |
| Gyro scale factor | 1.0 (no correction) | High — 0.749% error, within pass threshold |
| Mag hard-iron Y | −3.705e-05 T | High — machine-epsilon match |
| Mag hard-iron Z | +4.415e-05 T | High — machine-epsilon match |
| Mag soft-iron Y | 1.0958 | High — diff = 1.6e-5 |
| Mag soft-iron Z | 0.9196 | High — diff = 8e-6 |
| Mag horizontal axes | Y=col2, Z=col3 | High — confirmed by rotation Lissajous |
| R_mag = 100.0 | intentional workaround | Acknowledged — K_mag≈0, gyro dominates |
| R_tof = 0.01 | not verified in Session 0 | Defer to Sessions 1-5 |
| gamma_threshold = 3.84 | χ²₁ at p=0.05 | Correct formula; may be too tight in practice |
| Arena bounds | x∈[−1.2,1.2], y∈[−2.16,1.2] | Not verified from data — accept as given |
| ToF mounting angles | 0, π/2, −π/2 | Not verified from data — accept as given |
| Lateral axis | column 2 | High — confirmed in both stationary and motion analysis |

---

## Session 1 Priority

Fix BUGs 1, 2, and 3 together as a single atomic change — they are coupled (same axis swap). Do not fix one without the others or the bias residual will be different but still wrong. After the axis fix, re-evaluate Q_vx (currently 3.5e+06× overinflated) because the large Q was serving as a partial workaround for the broken accel integration.
