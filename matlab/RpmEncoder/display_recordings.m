clear;
clc;
close all;
format long g
sample = readtable('samples/test_record_rightmotor_2.csv');
sample2 = table2array(sample);

filtered_rpm = sample.filtered_rpm;
raw_throttle = sample.raw_throttle;
throttle = sample.throttle;
raw_rpm = sample.raw_rpm;
adjusted_rpm = sample.adjusted_rpm;
timestamp = 0:0.01:(length(raw_rpm)*0.01) - 0.01;
timestamp = timestamp';

skipSamples =  1; % 4050

adjusted_rpm = adjusted_rpm(skipSamples:end, 1);
throttle = throttle(skipSamples:end, 1);
timestamp = timestamp(skipSamples:end, 1);

hold on;

plot(timestamp,raw_rpm, 'Color', 'blue', 'DisplayName','raw rpm')
plot(timestamp,adjusted_rpm, 'Color', 'red', 'DisplayName','adjusted rpm')
plot(timestamp,raw_throttle, 'Color', 'black', 'DisplayName','raw throttle')
plot(timestamp,throttle, 'Color', 'green', 'DisplayName','throttle')


hold on;
legend
xlabel('time [s]')

ylabel('RPM')
