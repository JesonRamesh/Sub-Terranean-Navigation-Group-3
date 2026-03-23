function [X_Est, P_Est] = myEKF(out)
% myEKF - Extended Kalman Filter for 2D robot localisation inside a walled arena.
%
%   [X_Est, P_Est] = myEKF(out)
%
%   Inputs:
%       out   - Struct of logged sensor data (IMU, magnetometer, ToF, ground truth)
%   Outputs:
%       X_Est - [N x 5] state estimates [x, y, theta_world, vx, vy] (theta = GT yaw)
%       P_Est - [5 x 5 x N] state covariance matrices at each IMU timestep
%
%   Internal state X: [x; y; theta_sensor; vx; vy]. Output column 3 is theta_world.
%   vx, vy are body-frame (forward/lateral in the board Y-Z plane). Sensors fused:
%   Gyro (heading), Accel Y-Z (body velocity -> world position via theta_world),
%   Magnetometer (heading — near-disabled, ellipticity=0.36 makes it unreliable),
%   3x ToF (range).
%
%   BOARD ORIENTATION (confirmed by calib1_rotate.mat mag axis-pair scatter):
%   The mag Y-Z pair forms a complete circle during horizontal rotation.
%   Board X is therefore the vertical axis (world +Z).
%     Board X --> World +Z  (vertical, up)
%     Board Y --> horizontal plane axis 1
%     Board Z --> horizontal plane axis 2
%   Gyro yaw rate = gyro(:,1)  [Board X = world +Z, CCW positive]
%   Mag heading   = atan2(mag(:,3), mag(:,2))  [Board Z, Board Y]
%
%   HEADING OFFSET STRATEGY:
%   heading_plane_offset is set to 0 and theta_sensor0 is seeded directly
%   from the GT quaternion yaw. This avoids using the magnetometer (ellipticity
%   = 0.36) to bootstrap the offset, which previously caused a ~1 rad error.
%   Gyro bias drift = 0.001928 * 52s = ~0.1 rad over the full run — reliable.
%   Magnetometer is retained but near-disabled (R_mag = 200) so it provides
%   only a tiny residual correction without injecting distortion noise.

    %% =========================================================================
    %  1. DATA EXTRACTION
    % ==========================================================================
    imu_time  = out.Sensor_GYRO.time;
    gyro_data = squeeze(out.Sensor_GYRO.signals.values)';  % [N x 3] rad/s

    if isfield(out, 'Sensor_ACCEL')
        accel_time = out.Sensor_ACCEL.time(:);
        accel_data = squeeze(out.Sensor_ACCEL.signals.values)';
        if size(accel_data, 1) == 3, accel_data = accel_data'; end
    else
        accel_time = [];
        accel_data = [];
    end

    mag_time  = out.Sensor_MAG.time;
    mag_data  = squeeze(out.Sensor_MAG.signals.values)';   % [M x 3] Tesla

    tof_times = {out.Sensor_ToF1.time, out.Sensor_ToF2.time, out.Sensor_ToF3.time};
    tof_raw   = {squeeze(out.Sensor_ToF1.signals.values), ...
                 squeeze(out.Sensor_ToF2.signals.values), ...
                 squeeze(out.Sensor_ToF3.signals.values)};

    for i = 1:3
        if size(tof_raw{i}, 1) == 4, tof_raw{i} = tof_raw{i}'; end
    end

    % Ground truth used ONLY for initial pose — never fused into the filter
    gt_pos = squeeze(out.GT_position.signals.values);
    if size(gt_pos, 1) == 3, gt_pos = gt_pos'; end

    %% =========================================================================
    %  2. CALIBRATION CONSTANTS
    %     All values derived offline by calibrate_sensors.m.
    %     To update: re-run calibrate_sensors.m and paste the output block here.
    % ==========================================================================

    % ----- Gyroscope (calib2_straight.mat stationary window) -----------------
    gyro_bias  = [0.001928, -0.011091, -0.001405]; % [X, Y, Z] rad/s — X is yaw axis
    gyro_scale = 1.15;                            % dimensionless (calib1_rotate.mat)

    % ----- Accelerometer (calib2_straight.mat stationary window) -------------
    accel_bias = [0.0, 0.0, 0.0]; % [X, Y, Z] m/s^2 — Y,Z drive body forward/lateral velocity

    % ----- Magnetometer (calib1_rotate.mat, horizontal axes Y=2 and Z=3) -----
    % NOTE: ellipticity = 0.36 — well above the 0.05 target. Magnetometer is
    % unreliable for heading. R_mag is set to 200 so it contributes essentially
    % nothing (K_mag < 0.001). Retained only for minor long-term drift insurance.
    mag_hard = [-3.7050e-05,  4.4150e-05]; % [offset_Y, offset_Z] Tesla
    mag_soft = [1.0958, 0.9196];           % [scale_Y,  scale_Z]  dimensionless

    % ----- ToF noise (calib2_straight.mat stationary variance) ---------------
    R_tof = 0.000075;  % mean variance across all three sensors [m^2]
                       % individual std: ToF1=0.00821m  ToF2=0.00885m  ToF3=0.00882m

    % ----- EKF tuning ---------------------------------------------------------
    % R_mag: near-disabled. Magnetometer ellipticity = 0.36 (target < 0.05).
    % At R_mag = 200, K_mag = P(3,3)/(P(3,3)+200) << 0.001 at all times.
    % Gyro dominates heading; bad mag readings cannot inject significant error.
    % Previously R_mag = 0.5 caused 1+ rad of heading error from distorted readings.
    R_mag = 5.0;

    % Q: process noise.
    % Q(3,3) = 5e-4: balanced between 1e-7 (too tight — starved Kalman gain) and
    % 1e-3 (too loose — let noisy mag corrections dominate). Gyro-integrated heading
    % stays authoritative while P(3,3) remains large enough for ToF to nudge theta.
    Q = diag([1e-4, 1e-4, 2e-3, 0.8, 0.8]);

    % Innovation gates for ToF: chi2(0.99,1) statistical + hard absolute cap
    % Rejects wall-hole false readings and large outliers
    gate_chi2 = 4.0;   % chi-squared threshold
    gate_abs  = 0.3;   % absolute cap [m]

    % Velocity decay: first-order fade between 10 Hz ToF corrections
    % tau ~ 0.3s: exp(-dt/tau) ~ exp(-0.01/0.3) at ~100 Hz IMU rate
    vel_decay = 0.975;

    % ----- Arena geometry -----------------------------------------------------
    bounds    = struct('x_max', 1.2, 'x_min', -1.2, 'y_max', 1.65, 'y_min', -1.65);
    alpha_tofs = [0, -pi/2, pi/2];  % ToF1=forward, ToF2=left, ToF3=right [rad]

    % Dead reckoning test: set false for prediction + mag only (no ToF).
    enable_tof_updates = true;

    %% =========================================================================
    %  3. FILTER INITIALISATION
    %
    %   FIX: heading_plane_offset is set to 0 and theta_sensor0 is seeded
    %   directly from the GT quaternion yaw — no magnetometer involvement.
    %
    %   Previously, the offset was estimated as circmean(z_mag - theta_quat),
    %   but with ellipticity = 0.36 the magnetometer atan2 reading can be off
    %   by over 1 rad, yielding heading_plane_offset ~ -2.59 instead of
    %   the expected ~-pi/2. This caused a persistent ~1 rad bias throughout
    %   the entire run. Seeding theta_sensor0 = theta_quat_mean with offset = 0
    %   breaks that circular dependency cleanly.
    % ==========================================================================
    gt_rot = squeeze(out.GT_rotation.signals.values);
    if size(gt_rot, 1) == 4, gt_rot = gt_rot'; end

    N_init = max(1, min([50, size(gt_rot, 1), size(gt_pos, 1)]));

    if isfield(out.GT_rotation, 'time')
        gt_t = out.GT_rotation.time(:);
    elseif isfield(out.GT_position, 'time')
        gt_t = out.GT_position.time(:);
    else
        gt_t = imu_time(1:min(length(imu_time), size(gt_rot, 1)));
    end

    % Compute circular mean of GT quaternion yaw over N_init samples
    theta_quat_s = zeros(N_init, 1);
    for k = 1:N_init
        qw = gt_rot(k, 1); qx = gt_rot(k, 2); qy = gt_rot(k, 3); qz = gt_rot(k, 4);
        theta_quat_s(k) = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2));
    end
    theta_quat_mean = atan2(mean(sin(theta_quat_s)), mean(cos(theta_quat_s)));

    % FIX: seed theta_sensor0 directly from GT — no magnetometer bootstrap.
    % heading_plane_offset = 0 means gyro frame == world frame at initialisation.
    heading_plane_offset = 0;
    theta_sensor0        = theta_quat_mean;

    pos0 = mean(gt_pos(1:N_init, 1:2), 1);
    X    = [pos0(1); pos0(2); theta_sensor0; 0; 0];

    fprintf('myEKF: N_init=%d, heading_plane_offset=%.4f rad (fixed to 0 — GT-seeded)\n', ...
        N_init, heading_plane_offset);
    fprintf('myEKF: theta_sensor0=%.4f rad (from GT quaternion mean yaw)\n', theta_sensor0);

    % Tight on position/heading (seeded from GT), loose on velocity (unknown)
    P = diag([0.01, 0.01, 0.01, 0.1, 0.1]);

    %% =========================================================================
    %  4. ALLOCATE OUTPUT ARRAYS
    % ==========================================================================
    num_steps  = length(imu_time);
    X_Est      = zeros(num_steps, 5);
    P_Est      = zeros(5, 5, num_steps);
    mag_idx    = 1;
    tof_idxs   = [1, 1, 1];
    prev_time  = imu_time(1);
    mag_offset = [];  % magnetic-north-to-arena-frame offset, set on first mag sample

    %% =========================================================================
    %  5. MAIN EKF LOOP
    % ==========================================================================
    for k = 1:num_steps

        dt = imu_time(k) - prev_time;
        if dt <= 0, dt = 0.001; end
        prev_time = imu_time(k);

        % -----------------------------------------------------------------------
        % 5a. PREDICTION — gyro (yaw) + horizontal accel (body Y/Z -> velocity)
        %     Yaw rate = Board X (index 1). Body forward/lateral accel on Y,Z
        %     integrate into X(4:5), then map to world with theta_world for X(1:2).
        % -----------------------------------------------------------------------
        omega = gyro_scale * (gyro_data(k, 1) - gyro_bias(1));
        tw    = wrapToPi(X(3) - heading_plane_offset);  % theta_world = X(3) since offset=0

        if ~isempty(accel_data)
            [~, ia] = min(abs(accel_time - imu_time(k)));
            ay = accel_data(ia, 2) - accel_bias(2);
            az = accel_data(ia, 3) - accel_bias(3);
            a_fwd = ay;
            a_lat = az;
        else
            a_fwd = 0;
            a_lat = 0;
        end

        vx_pred = vel_decay * (X(4) + a_fwd * dt);
        vy_pred = vel_decay * (X(5) + a_lat * dt);

        X(1) = X(1) + (vx_pred * cos(tw) - vy_pred * sin(tw)) * dt;
        X(2) = X(2) + (vx_pred * sin(tw) + vy_pred * cos(tw)) * dt;
        X(3) = wrapToPi(X(3) + omega * dt);
        X(4) = vx_pred;
        X(5) = vy_pred;

        % Linearised motion Jacobian F = d(f)/d(X); tw = X(3) - heading_plane_offset
        vd = vel_decay;
        F  = eye(5);
        F(1, 3) = (-vx_pred * sin(tw) - vy_pred * cos(tw)) * dt;
        F(1, 4) =  cos(tw) * vd * dt;
        F(1, 5) = -sin(tw) * vd * dt;
        F(2, 3) =  (vx_pred * cos(tw) - vy_pred * sin(tw)) * dt;
        F(2, 4) =  sin(tw) * vd * dt;
        F(2, 5) =  cos(tw) * vd * dt;
        F(4, 4) = vd;
        F(5, 5) = vd;

        P = F * P * F' + Q;

        % -----------------------------------------------------------------------
        % 5b. MAGNETOMETER UPDATE — near-disabled heading correction at ~50 Hz
        %     R_mag = 200 means K_mag << 0.001 at all times — gyro dominates.
        %     Horizontal axes: Board Y (index 2) and Board Z (index 3).
        %     Board X (index 1) is vertical — not used for heading.
        %     mag_offset aligns the magnetometer's arbitrary zero to theta_sensor0
        %     so the residual correction (when it does fire) is in the right direction.
        % -----------------------------------------------------------------------
        if mag_idx <= length(mag_time) && imu_time(k) >= mag_time(mag_idx)

            my    = (mag_data(mag_idx, 2) - mag_hard(1)) * mag_soft(1);  % Board Y
            mz    = (mag_data(mag_idx, 3) - mag_hard(2)) * mag_soft(2);  % Board Z
            z_mag = wrapToPi(atan2(mz, my));

            if isempty(mag_offset)
                % Align magnetometer zero to current theta_sensor0 on first reading
                mag_offset = wrapToPi(theta_sensor0 - z_mag);
            end

            inn_mag = wrapToPi((z_mag + mag_offset) - X(3));
            H_mag   = [0, 0, 1, 0, 0];
            S_mag   = H_mag * P * H_mag' + R_mag;
            K_mag   = P * H_mag' / S_mag;
            X       = X + K_mag * inn_mag;
            X(3)    = wrapToPi(X(3));

            I_KH = eye(5) - K_mag * H_mag;
            P    = I_KH * P * I_KH' + K_mag * R_mag * K_mag';

            mag_idx = mag_idx + 1;
        end

        % -----------------------------------------------------------------------
        % 5c. ToF UPDATES — position correction at ~10 Hz per sensor
        %     Ray-cast predicts expected wall distance from current pose estimate.
        %     Dual gate rejects wall-hole false readings before Kalman update.
        %     Skipped when enable_tof_updates is false (dead-reckoning test).
        % -----------------------------------------------------------------------
        if enable_tof_updates
            for i = 1:3
                if tof_idxs(i) <= length(tof_times{i}) && ...
                   imu_time(k) >= tof_times{i}(tof_idxs(i))

                    data = tof_raw{i}(tof_idxs(i), :);

                    if data(4) == 0 && data(1) > 0.05 && data(1) < 4.0
                        [h, H] = ray_cast(X, bounds, alpha_tofs(i), heading_plane_offset);

                        if h > 0 && h < 5.0
                            inn   = data(1) - h;
                            S_tof = H * P * H' + R_tof;

                            if (inn^2 / S_tof) < gate_chi2 && abs(inn) < gate_abs
                                K_tof = P * H' / S_tof;
                                X     = X + K_tof * inn;
                                X(3)  = wrapToPi(X(3));

                                I_KH = eye(5) - K_tof * H;
                                P    = I_KH * P * I_KH' + K_tof * R_tof * K_tof';
                            end
                        end
                    end

                    tof_idxs(i) = tof_idxs(i) + 1;
                end
            end
        end

        X_out        = X;
        X_out(3)     = wrapToPi(X(3) - heading_plane_offset);  % = X(3) since offset=0
        X_Est(k, :)  = X_out';
        P_Est(:, :, k) = P;

    end % main EKF loop

end % myEKF


%% ============================================================================
%  HELPER FUNCTIONS
% =============================================================================

function [h, H] = ray_cast(X, b, alpha, heading_plane_offset)
% RAY_CAST  Expected ToF distance to nearest arena wall + measurement Jacobian.
%   Casts a ray from [X(1), X(2)] in direction theta_world+alpha and returns the
%   shortest positive intersection h with the four arena walls, plus H = d(h)/d(X).
%   theta_world = wrapToPi(X(3) - heading_plane_offset).

    if nargin < 4, heading_plane_offset = 0; end
    phi = wrapToPi(X(3) - heading_plane_offset + alpha);
    cp  = cos(phi);
    sp  = sin(phi);

    d = [inf, inf, inf, inf];
    if  cp >  1e-6,  d(1) = (b.x_max - X(1)) / cp; end  % +X wall
    if  cp < -1e-6,  d(2) = (b.x_min - X(1)) / cp; end  % -X wall
    if  sp >  1e-6,  d(3) = (b.y_max - X(2)) / sp; end  % +Y wall
    if  sp < -1e-6,  d(4) = (b.y_min - X(2)) / sp; end  % -Y wall

    [h, idx] = min(d);

    H = zeros(1, 5);
    if idx <= 2
        H(1) = -1 / cp;         % dh/dx
        H(3) =  h * (sp / cp);  % dh/dtheta
    else
        H(2) = -1 / sp;         % dh/dy
        H(3) = -h * (cp / sp);  % dh/dtheta
    end
end


function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end