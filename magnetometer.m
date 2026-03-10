mag = squeeze(out.Sensor_MAG.signals.values);
time_m = out.Sensor_MAG.time;
figure;
plot(time_m, mag(1,:), 'r'); hold on;
plot(time_m, mag(2,:), 'g');
plot(time_m, mag(3,:), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Magnetic field (T)');
title('Magnetometer - Calib Rotate');

%%
hard_iron_y = (max(mag(2,:)) + min(mag(2,:))) / 2
hard_iron_z = (max(mag(3,:)) + min(mag(3,:))) / 2

mag_corrected = mag;
mag_corrected(2,:) = mag(2,:) - hard_iron_y;
mag_corrected(3,:) = mag(3,:) - hard_iron_z;

figure;
plot(time_m, mag_corrected(1,:), 'r'); hold on;
plot(time_m, mag_corrected(2,:), 'g');
plot(time_m, mag_corrected(3,:), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Magnetic field (T)');
title('Magnetometer - Hard Iron Corrected');

%%
figure;
plot(mag(2,:), mag(3,:), 'b.'); hold on;
plot(mag_corrected(2,:), mag_corrected(3,:), 'r.');
legend('Raw', 'Corrected');
xlabel('B_Y (T)');
ylabel('B_Z (T)');
title('Magnetometer Y-Z plane');

%%
amp_y = (max(mag_corrected(2,:)) - min(mag_corrected(2,:))) / 2
amp_z = (max(mag_corrected(3,:)) - min(mag_corrected(3,:))) / 2
avg_amp = (amp_y + amp_z) / 2

soft_iron_y = avg_amp / amp_y
soft_iron_z = avg_amp / amp_z
%%
mag_corrected(2,:) = mag_corrected(2,:) * soft_iron_y;
mag_corrected(3,:) = mag_corrected(3,:) * soft_iron_z;

figure;
plot(mag(2,:), mag(3,:), 'b.'); hold on;
plot(mag_corrected(2,:), mag_corrected(3,:), 'r.');
legend('Raw', 'Fully Corrected');
xlabel('B_Y (T)');
ylabel('B_Z (T)');
title('Magnetometer Y-Z plane - Full Correction');
axis equal;

%%
calib.mag.hard_iron_y = hard_iron_y;
calib.mag.hard_iron_z = hard_iron_z;
calib.mag.soft_iron_y = soft_iron_y;
calib.mag.soft_iron_z = soft_iron_z;