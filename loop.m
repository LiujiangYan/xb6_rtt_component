% sine sweep

start_freq = 1;
end_freq = 2;
amplitude = 2;
duration = 10;

start_angular_freq = 2*pi*start_freq;
end_angular_freq = 2*pi*end_freq;

K = (start_angular_freq*duration)/log(end_angular_freq/start_angular_freq);
L = (duration)/log(end_angular_freq/start_angular_freq);

pos = [];
sign = 1;
dt = 0;
offset = 0;

for i=1:2000
    if sign == 1  
        cmd = amplitude*sin(K*(exp((dt)/(L))-1)) + offset;
        dt = dt + 0.02; 
    end
    if sign == -1
        cmd = -amplitude*sin(K*(exp((dt)/(L))-1)) + offset;
        dt = dt - 0.02;  
    end
    pos = [pos;cmd];
    if dt>10 || dt<0
        offset = cmd;
        sign = -sign;
    end
end

plot(pos);