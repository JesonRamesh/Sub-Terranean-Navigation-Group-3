time = out.Sensor_GYRO.time;

figure;
plot(time, gyro(1,:), 'r'); hold on;
plot(time, gyro(2,:), 'g');
plot(time, gyro(3,:), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (rad/s)');
title('Gyroscope');


%% corrected gyro
stat_idx = time < 60;
gyro_bias = mean(gyro(:, stat_idx), 2);
gyro_corrected = gyro - gyro_bias;

figure;
plot(time, gyro_corrected(1,:), 'r'); hold on;
plot(time, gyro_corrected(2,:), 'g');
plot(time, gyro_corrected(3,:), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (rad/s)');
title('Gyroscope - Bias Corrected');

%%
gyro_noise_var = var(gyro_corrected(:, stat_idx), 0, 2)