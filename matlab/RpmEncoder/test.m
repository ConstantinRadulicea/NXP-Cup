clear;
clc;
format long g
sample = readtable('samples/speed_rightmotor_16.txt');

sample = table2array(sample);
sample = transpose(sample);

 sample = sample';
 sample = sample(~isnan(sample))';

x = [1:1:length(sample)];
sample_mean = mean(sample);
sample_std = std(sample);
sample_std_percent = sample_std / sample_mean;

% histogram(sample, 200);
% title('RMP sampling 50hz')
% xlabel('RPM') 
% ylabel('Count') 

plot(x, sample);
xlabel('time') 
ylabel('RPM') 

 gggg = (1/(500 * 60))*1000000