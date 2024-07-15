clear;
clc;
format long g
sample = readtable('samples/100speed_leftmotor_13.txt');

sample = table2array(sample);
sample = transpose(sample);
x = [1:1:length(sample)];
sample_mean = mean(sample);
sample_std = std(sample);
sample_std_percent = sample_std / sample_mean;

histogram(sample, 100);
title('RMP sampling 100hz')
xlabel('RPM') 
ylabel('Count') 

 plot(x, sample);

 gggg = (1/(500 * 60))*1000000