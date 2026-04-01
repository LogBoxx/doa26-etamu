% File: lidar_read_tf02_v5_5.m
% Purpose: TF02-Pro UART read with persistent connection and checksum validation.

function [dist_cm, strength, status] = lidar_read_tf02_v5_5(r, opts, command)

persistent dev;

dist_cm = NaN;
strength = NaN;
status = struct( ...
    'frame_ok', false, ...
    'checksum_ok', false, ...
    'reconnected', false, ...
    'temp_c', NaN, ...
    'message', "");

if nargin >= 3
    cmd = lower(string(command));
    if cmd == "reset"
        dev = [];
        status.message = "LiDAR UART state reset";
        return;
    end
end

if isempty(r)
    status.message = "Raspberry Pi object is required";
    return;
end

for attempt = 0:opts.serial_max_retries
    if isempty(dev)
        [dev, open_ok] = open_dev(r, opts);
        if ~open_ok
            status.message = "Failed to open serial device";
            continue;
        end
        if attempt > 0
            status.reconnected = true;
        end
    end

    try
        raw = read(dev, opts.serial_burst_bytes, 'uint8');
    catch read_ex
        status.message = string(read_ex.message);
        dev = [];
        continue;
    end

    [dist_cm, strength, frame_status] = parse_first_valid_frame(raw);
    status.checksum_ok = frame_status.checksum_ok;
    status.temp_c = frame_status.temp_c;

    if frame_status.frame_ok
        status.frame_ok = true;
        status.message = "OK";
        return;
    end

    status.message = frame_status.message;
    dev = [];
end
end

function [dev, ok] = open_dev(r, opts)
dev = [];
ok = false;
try
    dev = serialdev(r, opts.serial_port, opts.serial_baud, 8, 'none', 1);
    dev.Timeout = opts.serial_timeout_sec;
    ok = true;
catch
    ok = false;
end
end

function [dist_cm, strength, status] = parse_first_valid_frame(raw)
dist_cm = NaN;
strength = NaN;
status = struct('frame_ok', false, 'checksum_ok', false, 'temp_c', NaN, 'message', "No valid frame");

if isempty(raw) || numel(raw) < 9
    status.message = "Insufficient bytes";
    return;
end

idx = strfind(raw(:).', [0x59 0x59]);
if isempty(idx)
    status.message = "No TF02 header";
    return;
end

for k = 1:numel(idx)
    start_idx = idx(k);
    if start_idx + 8 > numel(raw)
        continue;
    end

    frame = raw(start_idx:start_idx+8);
    expected_cs = mod(sum(double(frame(1:8))), 256);
    if expected_cs ~= double(frame(9))
        continue;
    end

    status.frame_ok = true;
    status.checksum_ok = true;

    dist_cm = double(frame(3)) + bitshift(double(frame(4)), 8);
    strength = double(frame(5)) + bitshift(double(frame(6)), 8);
    temp_raw = double(frame(7)) + bitshift(double(frame(8)), 8);
    status.temp_c = temp_raw / 8 - 256;
    status.message = "OK";
    return;
end

status.message = "Header found but checksum invalid";
end
