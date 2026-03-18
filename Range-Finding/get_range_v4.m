% File: get_range_v4.m
% Requires: MATLAB Support Package for Raspberry Pi Hardware
% Usage:

function [dist_cm, strength] = get_range_v4(r, azimuth, elevation, s_az, s_el, max_step)
%get_range_v4  Read TF02-Pro and position the servos safely
%   [dist_cm,strength] = get_range_v4(r, azimuth, elevation, s_az,
%   s_el) moves the azimuth/elevation servos and reads from the TF02-Pro
%   ranging module.  azimuth is expected in degrees (0–359) even though the
%   physical azimuth servo only covers 0–180; the mapping below converts the
%   “wrapped” user angle to the correct servo position.  When elevation is
%   outside the allowed range it is clamped.
%
%   An optional sixth argument, max_step, limits how far the azimuth input
%   may change on each call.  This prevents the servo from trying to jump
%   across large spans or flip direction when crossing the 0/360 boundary.
%   If omitted a conservative default of 10° is used.

% track the previous input so that we can restrict the movement amount
persistent prevAz;

if nargin < 6 || isempty(max_step)
    max_step = 10;    % degrees per invocation (tune as needed)
end

if isempty(prevAz)
    % first call, just remember the value
    prevAz = azimuth;
else
    % compute signed minimal difference on the circle
    diff = mod(azimuth - prevAz + 180, 360) - 180;
    if abs(diff) > max_step
        % step only in the direction of 'diff'
        azimuth = prevAz + sign(diff)*max_step;
        azimuth = mod(azimuth, 360);
    end
    prevAz = azimuth;
end

%% Move servos to desired position (with constraints)
mapped = zeros(size(azimuth));     % preallocate
idx1 = (azimuth >= 270 & azimuth <= 359); % Valid region 1: 270–359
mapped(idx1) = azimuth(idx1) - 270;
idx2 = (azimuth >= 0 & azimuth <= 90); % Valid region 2: 0–89
mapped(idx2) = azimuth(idx2) + 90; 
idx1_invalid = (azimuth < 270 & azimuth >= 180); % mid‑zone – hold at 0
mapped(idx1_invalid) = 0;
idx2_invalid = ~(idx1 | idx2 | idx1_invalid);
mapped(idx2_invalid) = 179;




azimuth = mapped;


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
        fprintf("DoAing Capstone Day")
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
