load('Training Data/task2_3 1.mat');  % loads 'out'
s.Sensor_ACCEL = out.Sensor_ACCEL;
s.Sensor_GYRO = out.Sensor_GYRO;
s.Sensor_MAG = out.Sensor_MAG;
s.Sensor_Time = out.Sensor_Time;
s.Sensor_ToF1 = out.Sensor_ToF1;
s.Sensor_ToF2 = out.Sensor_ToF2;
s.Sensor_ToF3 = out.Sensor_ToF3;
s.GT_position = out.GT_position;
s.GT_rotation = out.GT_rotation;
s.GT_time = out.GT_time;
s.Sensor_Temp = out.Sensor_Temp;
s.Sensor_LP_ACCEL = out.Sensor_LP_ACCEL;
save('task2_3_extracted.mat', '-struct', 's', '-v7');