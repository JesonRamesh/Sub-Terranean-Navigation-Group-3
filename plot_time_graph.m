imu_time = out.Sensor_GYRO.time;
gyro_data = squeeze(out.Sensor_GYRO.signals.values)';
accel_data = squeeze(out.Sensor_ACCEL.signals.values)';

figure(4);
plot(imu_time, gyro_data);

figure(5);
plot(imu_time, accel_data);