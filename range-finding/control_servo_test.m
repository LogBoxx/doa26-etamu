clear r;
r = raspi('169.254.52.8','analog','analog');
    if ~exist('s_az', 'var')
    s_az = servo(r, 12, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);  
    end
    if ~exist('s_el', 'var')
    s_el = servo(r, 13, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);  
    end
    dev = serialdev(r, '/dev/serial0', 115200, 8, 'none', 1);
    dev.Timeout = 0.05;


% 4) Sweep example
for ang = 0:5:160
    get_range_v1(r,ang,170,dev,s_az,s_el)
end

% get_range_v0(r,50,120);
% clear;
