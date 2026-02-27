% Demo: Move servos to several az/el points, then read TF02-Pro once per point.
% Files required on path: get_range_v5.m, tf02_read_once.m

clear; clc;

%% --- Raspberry Pi and servo setup ---
% Adjust IP/user/pass and GPIOs to your actual setup
r = raspi('169.254.52.8', 'analog', 'analog');
dev = serialdev(r, '/dev/serial0', 115200, 8, 'none', 1);
dev.Timeout = 0.1;
% Example: GPIO18 for azimuth, GPIO19 for elevation (change as needed)
% If you use specific pulse widths, set them here. Otherwise, default works if already tuned.
if ~exist('s_az', 'var')
    s_az = servo(r, 12, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);  
end
if ~exist('s_el', 'var')
    s_el = servo(r, 13, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);  
end
% Optional: go to a known safe/idle position at start
% writePosition(s_az, 90);  % use your mapping convention if needed
% writePosition(s_el, 120);

%% --- Test points (replace with your DOA outputs if desired) ---
% azimuth in 0..359 (your function maps 270..359->0..89 and 0..89->89..178, else ->179)
% elevation constrained to 110..175 inside the function
az_list = [350, 10, 30, 280, 320, 60, 100];  % includes some invalid-range cases to test mapping
el_list = [115, 120, 150, 170, 175, 140, 90]; % 90 will clamp to 110

% Sanity: make sizes match
assert(numel(az_list) == numel(el_list), 'az_list and el_list must match in length.');

%% --- Sweep ---
fprintf('Starting sweep (%d points)...\n', numel(az_list));

for k = 1:numel(az_list)
    az = az_list(k);
    el = el_list(k);

    % Move servos (with your mapping/limits) then read LiDAR once (persistent handle inside)
    [dist_cm, strength, temp_C] = get_range_v4(r, az, el, s_az, s_el);

    % Print concise result
    fprintf('k=%2d | in(az=%3d, el=%3d) -> dist=%.1f cm, S=%d, T=%.1f C\n', ...
        k, az, el, dist_cm, strength, temp_C);

    % Small pacing so you can see movement; reduce/increase as you like
    pause(0.05);
end

%% --- Done: release (optional) ---
% Move to idle or safe position if needed
% writePosition(s_az, 179);
% writePosition(s_el, 110);

fprintf('Sweep complete.\n');