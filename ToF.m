tof1 = squeeze(out.Sensor_ToF1.signals.values);
time_t = out.Sensor_ToF1.time;
size(tof1)
%%
tof1_dist   = tof1(:, 1);
tof1_status = tof1(:, 4);

% Only keep valid readings
valid = tof1_status == 0;
tof1_valid = tof1_dist(valid);

figure;
plot(time_t, tof1_dist, 'b.');
xlabel('Time (s)');
ylabel('Distance (m)');
title('ToF1 - Raw Distance');
%%
stat_idx_t = time_t < 60;
tof1_stat  = tof1_dist(stat_idx_t & valid);

tof1_mean = mean(tof1_stat)
tof1_var  = var(tof1_stat)
tof1_std  = std(tof1_stat)

%%
tof2 = squeeze(out.Sensor_ToF2.signals.values);
tof3 = squeeze(out.Sensor_ToF3.signals.values);

tof2_dist   = tof2(:, 1);
tof2_status = tof2(:, 4);
tof3_dist   = tof3(:, 1);
tof3_status = tof3(:, 4);

valid2 = tof2_status == 0;
valid3 = tof3_status == 0;

stat_idx_t2 = time_t < 60;
tof2_stat = tof2_dist(stat_idx_t2 & valid2);
tof3_stat = tof3_dist(stat_idx_t2 & valid3);

tof2_mean = mean(tof2_stat)
tof2_var  = var(tof2_stat)
tof2_std  = std(tof2_stat)

tof3_mean = mean(tof3_stat)
tof3_var  = var(tof3_stat)
tof3_std  = std(tof3_stat)

%%
calib.tof.mean   = [tof1_mean; tof2_mean; tof3_mean];
calib.tof.var    = [tof1_var;  tof2_var;  tof3_var];
calib.tof.std    = [tof1_std;  tof2_std;  tof3_std];