
in = throttle;
out = adjusted_rpm;
fs = 1/0.01;

data_ = iddata(out, in, 1/fs);

np = 3;
% awd = tfest(data, np)
ISI_tf = ssest(data_, np)

pid_tuned = pidTuner(ISI_tf, 'PID')