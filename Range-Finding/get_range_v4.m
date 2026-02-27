function [dist_cm, strength, temp_C] = get_range_v4(r, azimuth, elevation, s_az, s_el)
% Move servos, then read TF02-Pro once using a persistent reader.

%% Map azimuth into 0..179 using your two valid windows
mapped = zeros(size(azimuth));     % preallocate

% Valid region 1: 270–359  -> 0..89
idx1 = (azimuth >= 270 & azimuth <= 359);
mapped(idx1) = azimuth(idx1) - 270;

% Valid region 2: 0–89     -> 89..178
idx2 = (azimuth >= 0   & azimuth <= 89);
mapped(idx2) = azimuth(idx2) + 89;

% Everything else → hold at 179
idx_invalid = ~(idx1 | idx2);
mapped(idx_invalid) = 179;

azimuth = mapped;

%% Elevation constraints
if elevation < 110
    elevation = 110;
elseif elevation > 175
    elevation = 175;
end

%% Write servos first (LiDAR read happens only after this)
writePosition(s_az, azimuth);
writePosition(s_el, elevation);

% Optional: very small settle can help, but keep minimal
% pause(0.02);

%% Single LiDAR read (persistent serial handle inside)
[dist_cm, strength, temp_C] = tf02_read_once(r);
end
