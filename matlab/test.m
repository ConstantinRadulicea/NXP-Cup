raw_rpm = [1];
sample_batch_size = length(raw_rpm);
sample_batch_max_size = 2;
sample_batch_size = min(sample_batch_size, sample_batch_max_size)
raw_rpm = raw_rpm(end - sample_batch_size + 1:end)



