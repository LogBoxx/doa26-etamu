% File: get_range_v5_5.m
% Purpose: Refactored tracker + LiDAR wrapper with configurable options.
%
% Usage:
%   [dist_cm, strength, cmd_az, cmd_el, status] = get_range_v5_5(r, az_in, el_in, s_az, s_el)
%   [dist_cm, strength, cmd_az, cmd_el, status] = get_range_v5_5(r, az_in, el_in, s_az, s_el, opts)
%   get_range_v5_5('reset')

function [dist_cm, strength, cmd_az, cmd_el, status] = get_range_v5_5(r, azimuth, elevation, s_az, s_el, opts_in)

persistent tracker_state;
persistent conf_state;
persistent t0;
persistent last_t;

dist_cm = NaN;
strength = NaN;
cmd_az = NaN;
cmd_el = NaN;
status = struct();

if nargin >= 1 && (ischar(r) || isstring(r))
    cmd = lower(string(r));
    if cmd == "reset"
        % Reset both tracker state and UART handle state.
        tracker_state = [];
        conf_state = [];
        t0 = [];
        last_t = [];
        lidar_read_tf02_v5_5([], default_opts(), "reset");
        status.reset = true;
        status.message = "Tracker and LiDAR states reset";
        return;
    end
end

if nargin < 5
    error('get_range_v5_5 requires r, azimuth, elevation, s_az, s_el.');
end

if nargin < 6
    opts_in = struct();
end
opts = merge_opts(default_opts(), opts_in);

if isempty(conf_state)
    conf_state = opts.conf_init;
end
if isempty(t0)
    t0 = tic;
    last_t = 0;
end

% Monotonic time base for regression even if loop jitter occurs.
t_now = toc(t0);
if t_now <= last_t
    t_now = last_t + 1e-3;
end
last_t = t_now;

[tracker_state, cmd_az, cmd_el, track_status] = tracker_step_v5_5( ...
    tracker_state, azimuth, elevation, t_now, opts, conf_state);

% Hardware command output (degrees in your current servo setup).
writePosition(s_az, cmd_az);
writePosition(s_el, cmd_el);

[dist_cm, strength, lidar_status] = lidar_read_tf02_v5_5(r, opts);

if lidar_status.frame_ok
    raw_conf = strength_to_confidence(strength, opts);
    conf_state = opts.conf_ema_keep * conf_state + (1 - opts.conf_ema_keep) * raw_conf;
else
    conf_state = max(opts.conf_floor, opts.conf_fail_decay * conf_state);
end

status = struct();
status.confidence_next = conf_state;
status.track = track_status;
status.lidar = lidar_status;
status.options = opts;
end

function c = strength_to_confidence(strength, opts)
c = (double(strength) - opts.strength_min) / (opts.strength_max - opts.strength_min);
c = min(max(c, 0), 1);
end

function opts = default_opts()
opts = struct();

% Mapping/motion constraints
opts.lower_az = 10;
opts.upper_az = 169;
opts.lower_el = 15;
opts.upper_el = 70;
opts.max_move_az = 10;
opts.max_move_el = 5;

% Prediction model
opts.window_size = 6;
opts.min_points_for_fit = 4;
opts.predict_horizon_sec = 0.06;

% Confidence behavior
opts.alpha_lo = 0.25;
opts.alpha_hi = 0.85;
opts.conf_gate = 0.15;
opts.conf_init = 0.50;
opts.conf_floor = 0.10;
opts.conf_fail_decay = 0.80;
opts.conf_ema_keep = 0.70;

% Strength-to-confidence mapping
opts.strength_min = 1500;
opts.strength_max = 15000;

% UART settings
opts.serial_port = '/dev/serial0';
opts.serial_baud = 115200;
opts.serial_timeout_sec = 0.1;
opts.serial_burst_bytes = 36;
opts.serial_max_retries = 1;
end

function out = merge_opts(base, override)
out = base;
if isempty(override)
    return;
end
fn = fieldnames(override);
for i = 1:numel(fn)
    out.(fn{i}) = override.(fn{i});
end
end
