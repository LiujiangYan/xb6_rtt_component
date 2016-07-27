% sine sweep

start_freq = 1;
end_freq = 5;
amplitude = 2;
duration = 10;

start_angular_freq = 2*pi*start_freq;
end_angular_freq = 2*pi*end_freq;

K = (start_angular_freq*duration)/log(end_angular_freq/start_angular_freq);
L = (duration)/log(end_angular_freq/start_angular_freq);

pos = [];
t = 0:0.002:10;
t = t';
for dt=0:0.002:10
    cmd = amplitude*sin(K*(exp((dt)/(L))-1));
    pos = [pos;cmd];
end

plot(t, pos);