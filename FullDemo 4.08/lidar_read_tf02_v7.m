% File: lidar_read_tf02_v7.m
% Purpose: TF02-Pro UART read with persistent connection and checksum validation.

function [dist_cm, strength, status] = lidar_read_tf02_v7(r, opts, command)

persistent dev;

dist_cm = NaN;
strength = NaN;
status = struct( ...
    'frame_ok', false, ...
    'checksum_ok', false, ...
    'reconnected', false, ...
    'bytes_available', NaN, ...
    'bytes_discarded', 0, ...
    'bytes_read', 0, ...
    'valid_frames_in_burst', 0, ...
    'temp_c', NaN, ...
    'message', "");

if nargin >= 3
    cmd = lower(string(command));
    if cmd == "reset"
        % Drop UART handle so next read forces reopen.
        dev = [];
        status.message = "LiDAR UART state reset";
        return;
    end
end

if isempty(r)
    status.message = "Raspberry Pi object is required";
    return;
end

for attempt = 0:opts.serial_max_retries % opts.serial_max_retries: reconnect attempts per call
    if isempty(dev)
        % Reuse one persistent serial handle for lower latency.
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
        % Drain old backlog completely so output is based on fresh bytes only.
        bytes_avail = get_num_bytes_available(dev);
        status.bytes_available = bytes_avail;
        drained_total = 0;
        while bytes_avail > 0
            n_drain = min(bytes_avail, opts.serial_max_read_bytes); % opts.serial_max_read_bytes: cap chunk size while draining
            read(dev, n_drain, 'uint8');
            drained_total = drained_total + n_drain;
            bytes_avail = get_num_bytes_available(dev);
        end
        status.bytes_discarded = drained_total;

        % Capture a short fresh window and parse the latest valid frame from it.
        if isfield(opts, 'serial_fresh_window_sec')
            fresh_window_sec = opts.serial_fresh_window_sec;
        else
            fresh_window_sec = 0.02;
        end
        t_capture = tic;
        raw = uint8([]);
        while toc(t_capture) < fresh_window_sec
            bytes_now = get_num_bytes_available(dev);
            if bytes_now > 0
                n_read = min(bytes_now, opts.serial_max_read_bytes); % opts.serial_max_read_bytes: cap chunk size while capturing
                chunk = read(dev, n_read, 'uint8');
                raw = [raw; chunk(:)];
            else
                pause(0.001);
            end
        end

        % One final non-blocking pull of bytes that arrived at window end.
        bytes_tail = get_num_bytes_available(dev);
        if bytes_tail > 0
            n_tail = min(bytes_tail, opts.serial_max_read_bytes); % opts.serial_max_read_bytes: cap tail read size
            chunk_tail = read(dev, n_tail, 'uint8');
            raw = [raw; chunk_tail(:)];
        end

        status.bytes_read = numel(raw);
    catch read_ex
        status.message = string(read_ex.message);
        dev = [];
        continue;
    end

    [dist_cm, strength, frame_status] = parse_latest_valid_frame(raw);
    status.checksum_ok = frame_status.checksum_ok;
    status.temp_c = frame_status.temp_c;
    status.valid_frames_in_burst = frame_status.valid_frames_in_burst;

    if frame_status.frame_ok
        status.frame_ok = true;
        status.message = "OK";
        return;
    end

    status.message = frame_status.message;
    % Reopen on next attempt after invalid/incomplete frame sequence.
    dev = [];
end
end

function [dev, ok] = open_dev(r, opts)
dev = [];
ok = false;
try
    dev = serialdev(r, opts.serial_port, opts.serial_baud, 8, 'none', 1); % opts.serial_port/opts.serial_baud: UART device and baud
    dev.Timeout = opts.serial_timeout_sec; % opts.serial_timeout_sec: serial read timeout (s)
    ok = true;
catch
    ok = false;
end
end

function [dist_cm, strength, status] = parse_latest_valid_frame(raw)
dist_cm = NaN;
strength = NaN;
status = struct('frame_ok', false, 'checksum_ok', false, 'temp_c', NaN, 'message', "No valid frame", 'valid_frames_in_burst', 0);

if isempty(raw) || numel(raw) < 9
    status.message = "Insufficient bytes";
    return;
end

idx = strfind(raw(:).', [0x59 0x59]);
if isempty(idx)
    status.message = "No TF02 header";
    return;
end

last_valid_frame = [];
for k = 1:numel(idx)
    start_idx = idx(k);
    if start_idx + 8 > numel(raw)
        continue;
    end

    frame = raw(start_idx:start_idx+8);
    % TF02 checksum is sum of first 8 bytes modulo 256.
    expected_cs = mod(sum(double(frame(1:8))), 256);
    if expected_cs ~= double(frame(9))
        continue;
    end

    last_valid_frame = frame;
    status.valid_frames_in_burst = status.valid_frames_in_burst + 1;
end

if isempty(last_valid_frame)
    status.message = "Header found but checksum invalid";
    return;
end

status.frame_ok = true;
status.checksum_ok = true;
dist_cm = double(last_valid_frame(3)) + bitshift(double(last_valid_frame(4)), 8);
strength = double(last_valid_frame(5)) + bitshift(double(last_valid_frame(6)), 8);
temp_raw = double(last_valid_frame(7)) + bitshift(double(last_valid_frame(8)), 8);
status.temp_c = temp_raw / 8 - 256;
status.message = "OK";
end

function n = get_num_bytes_available(dev)
n = -1;
try
    if isprop(dev, 'NumBytesAvailable')
        n = double(dev.NumBytesAvailable);
    elseif isprop(dev, 'BytesAvailable')
        n = double(dev.BytesAvailable);
    end
catch
    n = -1;
end

if ~isfinite(n) || (n < 0)
    n = -1;
end
end
