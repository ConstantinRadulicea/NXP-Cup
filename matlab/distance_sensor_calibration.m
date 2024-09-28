clc;
clear;
close all;

real_distance_m = [0.07 0.1 0.14 0.16 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1 1.1 1.5];
sensor_data_m = [0.012 0.05 0.09 0.1 0.126 0.198 0.288 0.36 0.492 0.594 0.666 0.756 0.846 0.936 1.272];
offset_data = sensor_data_m - real_distance_m;
n_samples = length(real_distance_m);

plot(real_distance_m, sensor_data_m);
hold on;
plot([0, 1.5], [0, 1.5]);

plot(real_distance_m, offset_data);
hold on;


c = polyfit(sensor_data_m,offset_data,1);
y_est = polyval(c,sensor_data_m);
calibrated_data = sensor_data_m - y_est;
hold on;
plot(real_distance_m,y_est,'r--','LineWidth',2)

hold on;
plot(real_distance_m,calibrated_data,'LineWidth',2)
hold off

fprintf('calibrated_data = sensor_data - offset\n')
fprintf('offset = %.3f*sensor_data + %.3f\n',c)




