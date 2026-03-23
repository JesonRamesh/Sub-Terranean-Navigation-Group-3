%% Calibration - automatic stationary detection + robust magnetometer ellipsoid fit
% Assumes:
%  - calib2_straight.mat: robot stationary for ~1 min, plus straight drives.
%  - calib1_rotate.mat: robot rotated continuously (several full turns).
% Each file contains "out" structure with Sensor_* as in repository.

%% Parameters
min_stationary_duration = 5.0; % seconds minimum to consider a stationary window (use a chunk from the 1 min)
accel_std_threshold = 0.05;    % m/s^2 threshold to detect stationary (adjust if needed)
g = 9.80665;

%% Load files
C_stat = load('Training Data/calib2_straight.mat'); % stationary/straight run
C_rot  = load('Training Data/calib1_rotate.mat');  % rotation run

out_stat = C_stat.out;
out_rot  = C_rot.out;

%% Extract signals (robust shape handling)
getSignal = @(s) squeeze(s.signals.values);
% Gyro
gyro_raw = getSignal(out_stat.Sensor_GYRO);
t_g = out_stat.Sensor_GYRO.time(:);
% Accel
acc_raw = getSignal(out_stat.Sensor_ACCEL);
t_a = out_stat.Sensor_ACCEL.time(:);
% Magnetometer (rotation run)
mag_raw = getSignal(out_rot.Sensor_MAG);
t_m = out_rot.Sensor_MAG.time(:);

% ToF (optional)
hasToF1 = isfield(out_stat,'Sensor_ToF1');
if hasToF1, tof1 = getSignal(out_stat.Sensor_ToF1); t_tof = out_stat.Sensor_ToF1.time(:); end
hasToF2 = isfield(out_stat,'Sensor_ToF2'); if hasToF2, tof2 = getSignal(out_stat.Sensor_ToF2); end
hasToF3 = isfield(out_stat,'Sensor_ToF3'); if hasToF3, tof3 = getSignal(out_stat.Sensor_ToF3); end

% Ensure [N x 3]
toRows = @(X) (size(X,1)==3 && size(X,2)>3) * (X') + (size(X,1)~=3) * X;
if size(gyro_raw,1)==3 && size(gyro_raw,2)>3, gyro = gyro_raw'; else gyro = gyro_raw; end
if size(acc_raw,1)==3 && size(acc_raw,2)>3, acc = acc_raw'; else acc = acc_raw; end
if size(mag_raw,1)==3 && size(mag_raw,2)>3, mag = mag_raw'; else mag = mag_raw; end

%% Detect stationary window in calib2_straight (use accel low-variance window)
% Compute moving std of accel magnitude
acc_mag = sqrt(sum(acc.^2,2));
win = max(1, round(0.5 * mean(diff(t_a))^-1)); % approx 0.5s window (fallback)
% better: window samples ~ 1 sec
fs_a = 1/median(diff(t_a));
win = max(1, round(fs_a * 1.0));
movstd_acc = movstd(acc_mag, win);
% find contiguous regions where std below threshold
is_stat = movstd_acc < accel_std_threshold;
% find longest contiguous true segment
d = diff([0; is_stat; 0]);
start_idx = find(d==1);
end_idx   = find(d==-1)-1;
dur = (end_idx - start_idx + 1) ./ fs_a;
[best_dur, bi] = max(dur);
if best_dur >= min_stationary_duration
    s_idx = start_idx(bi);
    e_idx = end_idx(bi);
else
    % fallback: use initial 10 seconds if no long stationary region found
    s_idx = 1;
    e_idx = find(t_a <= (t_a(1) + 10), 1, 'last');
end
stat_idx_a = false(size(t_a)); stat_idx_a(s_idx:e_idx) = true;
% For gyro align timevector: find matching indices (interpolate by time)
stat_start_t = t_a(s_idx); stat_end_t = t_a(e_idx);
stat_idx_g = (t_g >= stat_start_t) & (t_g <= stat_end_t);
if exist('t_tof','var'), stat_idx_t = (t_tof >= stat_start_t) & (t_tof <= stat_end_t); end

%% Gyro units check + bias/noise
% convert deg->rad if large values suggest degrees
if max(abs(gyro(:))) > 50
    gyro = deg2rad(gyro);
    gyro_unit_note = 'deg/s converted to rad/s';
else
    gyro_unit_note = 'rad/s assumed';
end
gyro_bias = mean(gyro(stat_idx_g,:),1);
gyro_corr = bsxfun(@minus, gyro, gyro_bias);
gyro_noise_var = var(gyro_corr(stat_idx_g,:),0,1);

%% Accelerometer bias & noise
acc_mean = mean(acc(stat_idx_a,:),1);
g_mag = norm(acc_mean);
% static bias assuming gravity mainly in Z body
accel_static = acc_mean - [0,0,g_mag];
accel_noise_var = var(acc(stat_idx_a,:),0,1);

%% ToF stats if present
if hasToF1
    tof1_dist = tof1(:,1);
    tof1_status = size(tof1,2) >= 4 && ~isempty(tof1(:,4)) .* tof1(:,4); % handle missing field
    if size(tof1,2) >= 4
        valid1 = tof1(:,4) == 0;
    else
        valid1 = true(size(tof1_dist));
    end
    tof1_stat = tof1_dist(stat_idx_t & valid1);
    tof1_mean = mean(tof1_stat); tof1_var = var(tof1_stat);
else, tof1_mean = NaN; tof1_var = NaN; end
if hasToF2
    if size(tof2,2) >= 4, valid2 = tof2(:,4)==0; else valid2 = true(size(tof2,1),1); end
    tof2_stat = tof2(stat_idx_t & valid2,1); tof2_var = var(tof2_stat);
else, tof2_var = NaN; end
if hasToF3
    if size(tof3,2) >= 4, valid3 = tof3(:,4)==0; else valid3 = true(size(tof3,1),1); end
    tof3_stat = tof3(stat_idx_t & valid3,1); tof3_var = var(tof3_stat);
else, tof3_var = NaN; end

%% Magnetometer ellipsoid fit (algebraic fit -> center + transform)
% Solve quadratic form: x'*A*x + b'*x + c = 0  (see ellipsoid fit literature)
X = mag; % N x 3
D = [X(:,1).^2, X(:,2).^2, X(:,3).^2, 2*X(:,2).*X(:,3), 2*X(:,1).*X(:,3), 2*X(:,1).*X(:,2), 2*X(:,1), 2*X(:,2), 2*X(:,3), ones(size(X,1),1)];
% Solve normal equations
[~,S_diag,Vv] = svd(D,0);
% Use least-squares solution (more robust: use pinv)
coef = (D' * D) \ (D' * ones(size(X,1),1)); % 10x1
% Extract algebraic form
A_mat = [coef(1) coef(6) coef(5);
         coef(6) coef(2) coef(4);
         coef(5) coef(4) coef(3)];
b_vec = coef(7:9);
c_val = coef(10);
% center
center = -0.5 * (A_mat \ b_vec);
% Translate form to center
T = eye(3);
R = A_mat;
% compute translation-adjusted constant
c_center = center' * A_mat * center - c_val;
% eigen decomp for axes radii
[Ue,Se] = eig(R / c_center);
radii = sqrt(1 ./ diag(Se));
% soft-iron transform: map measured mag -> unit sphere
% transformation: x_cal = W * (x - center), where W = Ue * diag(1./radii) * Ue'
W = Ue * diag(1./radii) * Ue';
mag_hard = center';
mag_soft = W;

% apply calibration
Mc = bsxfun(@minus, mag, mag_hard);
M_cal = (mag_soft * Mc')';
% Compute calibrated-magnetometer radius robustly (handles complex components)
% Use per-component magnitude so r is real: r_i = sqrt( sum( |M_cal(i,:)|.^2 ) )
r = sqrt(sum(abs(M_cal).^2, 2));   % guaranteed real >= 0

% warn if there were non-negligible imaginary parts in M_cal
imag_max = max(abs(imag(M_cal(:))));
tol_imag_warn = 1e-6;
if imag_max > tol_imag_warn
    warning('Mag calibration produced non-negligible imaginary parts (max imag = %g). Using absolute-value radius.', imag_max);
end

% Remove non-finite entries just in case
r = r(isfinite(r));

% Make sure r is a column vector
r = r(:);

%% --- Validate M_cal is real; if complex, fallback to covariance SVD whitening ---
% M_cal currently computed from ellipsoid transform: M_cal = W * (mag - center)
if exist('M_cal','var')
    imag_max = max(abs(imag(M_cal(:))));
    tol_imag = 1e-8; % tolerance for numerical imaginary noise
    if imag_max > tol_imag
        warning('Mag calibration produced complex values (max imag = %g). Falling back to covariance/SVD soft-iron whitening.', imag_max);
        % Use simple hard-iron + covariance SVD whitening as fallback
        mag_hard = mean(mag,1);
        Mc = bsxfun(@minus, mag, mag_hard);
        [Uc,Sc,~] = svd(cov(Mc));
        mag_scale_diag = 1 ./ sqrt(diag(Sc));
        mag_soft = Uc * diag(mag_scale_diag) * Uc';
        M_cal = (mag_soft * Mc')';
        % drop any tiny imag parts
        M_cal = real(M_cal);
    else
        % Imag part is negligible -> drop numerical noise
        M_cal = real(M_cal);
    end
else
    error('M_cal is not defined at this point. Ensure magnetometer calibration ran before yaw calculation.');
end

% Final safety check: ensure M_cal is finite and real
if ~isreal(M_cal) || any(~isfinite(M_cal(:)))
    error('M_cal contains non-real or non-finite entries after cleanup. Aborting yaw computation.');
end

% Compute yaw (safe: inputs are real)
yaw_mag = atan2(M_cal(:,2), M_cal(:,1));

%% Print results
fprintf('=== Calibration Summary (auto) ===\n');
fprintf('Stationary window: t=%.2f..%.2f s (duration %.1fs)\n', stat_start_t, stat_end_t, stat_end_t-stat_start_t);
fprintf('Gyro units: %s\n', gyro_unit_note);
fprintf('Gyro bias (rad/s): X=%.6g Y=%.6g Z=%.6g\n', gyro_bias);
fprintf('Gyro var (stat):   X=%.6g Y=%.6g Z=%.6g\n', gyro_noise_var);
fprintf('Accel mean (m/s^2): X=%.6g Y=%.6g Z=%.6g\n', acc_mean);
fprintf('Accel static bias (m/s^2): X=%.6g Y=%.6g Z=%.6g\n', accel_static);
fprintf('Accel var (stat): X=%.6g Y=%.6g Z=%.6g\n', accel_noise_var);
fprintf('Mag hard-iron (3-axis): X=%.6g Y=%.6g Z=%.6g\n', mag_hard);
fprintf('Mag radius after transform: mean=%.6g std=%.6g\n', r_mean, r_std);
fprintf('ToF std (m): ToF1=%.6g ToF2=%.6g ToF3=%.6g\n', sqrt(tof1_var), sqrt(tof2_var), sqrt(tof3_var));

%% Plots for diagnostics
figure('Name','Calibration diagnostics','NumberTitle','off');
subplot(3,2,1); plot(t_g, gyro); title('Gyro raw'); xlabel('s'); legend('gx','gy','gz');
subplot(3,2,2); plot(t_a, acc); title('Accel raw'); xlabel('s'); legend('ax','ay','az');
subplot(3,2,3); plot3(mag(:,1), mag(:,2), mag(:,3), '.'); axis equal; title('Raw mag');
subplot(3,2,4); plot3(M_cal(:,1), M_cal(:,2), M_cal(:,3), '.'); axis equal; title('Calibrated mag');
subplot(3,2,5); histogram(r,50); title('Calibrated mag radius');
subplot(3,2,6); plot(t_m, wrapToPi(yaw_mag)); title('Mag-derived yaw');

%% Save constants
save('calibration_constants.mat','gyro_bias','gyro_noise_var','accel_static','accel_noise_var', ...
    'mag_hard','mag_soft','r_mean','r_std','tof1_var','tof2_var','tof3_var');
