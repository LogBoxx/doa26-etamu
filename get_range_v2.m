% File: get_range_v0.m
% Requires: MATLAB Support Package for Raspberry Pi Hardware
% Usage:

function [dist_cm, strength] = get_range_v2(r, azimuth,elevation,dev,s_az,s_el)

%% Move servos to desired position (with constraints)



    % Constrain azimuth to valid range [5, 179]
    if azimuth < 5
        azimuth = 5;
    elseif azimuth > 179
        azimuth = 179;
    end
    
    % Constrain elevation to valid range [110, 179]
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
  % very small; we only need a few bytes
    %read(dev,256,'uint8');
    % Read 2–3 frames worth of data at once (burst read)
    raw = read(dev, 27, 'uint8');   % 27 bytes = 3 frames (3×9)
    % Find first header 0x59 0x59
    idx = strfind(raw(:).', [0x59 0x59]);
    
    if isempty(idx)
        fprintf("No header found.\n");
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
    fprintf("Shit Really Fucked Now Buddy");
    fprintf(exception);
end
    clear dev;
end
