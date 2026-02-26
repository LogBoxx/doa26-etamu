% File: get_range_v0.m
% Requires: MATLAB Support Package for Raspberry Pi Hardware
% Usage:

function [dist_cm, strength] = get_range_v3(r, azimuth,elevation,s_az,s_el)

%% Move servos to desired position (with constraints)
mapped = zeros(size(azimuth));     % preallocate

% Valid region 1: 270–359
idx1 = (azimuth >= 270 & azimuth <= 359);
mapped(idx1) = azimuth(idx1) - 270;

% Valid region 2: 0–89
idx2 = (azimuth >= 0 & azimuth <= 89);
mapped(idx2) = azimuth(idx2) + 90;

% Everything else → hold at 180
idx_invalid = ~(idx1 | idx2);
mapped(idx_invalid) = 179;

azimuth = mapped;

% Keep your elevation constraints as-is
if elevation < 110
    elevation = 110;
elseif elevation > 175
    elevation = 175;
end

    % Set servo positions
    writePosition(s_az, azimuth);
    writePosition(s_el, elevation);
%% TF02-Pro UART Read
try
    dev = serialdev(r, '/dev/serial0', 115200, 8, 'none', 1);
    dev.Timeout = 0.1;

    % read(dev,256,'uint8');
  
    % Read 2–3 frames worth of data at once (burst read)
    raw = read(dev, 19, 'uint8');   % 27 bytes = 3 frames (3×9)
    idx = strfind(raw(:).', [0x59 0x59]);    % Find first header 0x59 0x59
    
    if isempty(idx)
        fprintf("No header found.\n");
        clear dev; clear r;
        r = raspi('169.254.52.8','analog','analog');
        dev = serialdev(r, '/dev/serial0', 115200, 8, 'none', 1);
        dev.Timeout = 0.1;
        raw = read(dev, 19, 'uint8'); 
        idx = strfind(raw(:).', [0x59 0x59]);
    end
    if isempty(idx)
        fprintf("fucking ass bitgch")
        return;
    end 

    % Extract exactly one 9‑byte frame
    start = idx(1);
    if start+8 > length(raw)
        fprintf("Incomplete frame.\n");
        return;
    end
    
    rest = raw(start+2:start+8);
    
    % Decode TF02 9‑byte format
    
    % [0]=59 [1]=59 [2]=Dist_L [3]=Dist_H [4]=Str_L [5]=Str_H [6]=Temp_L [7]=Temp_H [8]=CS
    dist_cm = double(rest(1)) + bitshift(double(rest(2)), 8);
    strength = double(rest(3)) + bitshift(double(rest(4)), 8);
    temp_raw = double(rest(5)) + bitshift(double(rest(6)), 8);
    temp_C   = temp_raw/8 - 256;   % from TF02 manual
    
    fprintf("Dist = %.1f cm   Strength = %d   Temp = %.1f C\n", dist_cm, strength, temp_C); 
 
 catch exception
    fprintf('%s\n', exception.message);
end
    clear dev;
end
