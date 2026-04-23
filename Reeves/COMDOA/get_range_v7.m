% File: get_range_v7.m
% Purpose: Refactored tracker + LiDAR wrapper with configurable options.
%
% Usage:
%   [dist_cm, strength, cmd_az, cmd_el, status] = get_range_v7(r, az_in, el_in, s_az, s_el)
%   [dist_cm, strength, cmd_az, cmd_el, status] = get_range_v7(r, az_in, el_in, s_az, s_el, opts)
%   get_range_v7('reset')
%====================== put lines under this into real demo=========
% r = raspi('169.254.52.8','analog','analog');
% s_az = servo(r,13,'MinPulseDuration',5.44e-4,'MaxPulseDuration',2.40e-3);
% s_el = servo(r,12,'MinPulseDuration',5.44e-4,'MaxPulseDuration',2.40e-3);
% 
% opts = struct();
% opts.el_offset = 0;      % tune if needed
% opts.max_move_az = 10;   % tune if needed
% opts.max_move_el = 5;    % tune if needed
% 
% get_range_v7('reset');
% while true
%     az_in = est_azimuth;      % your estimator output (pre +90 mapping)
%     el_in = est_elevation;    % your estimator output (el_offset applied internally)
% 
%     [dist_cm, strength, cmd_az, cmd_el, status] = ...
%         get_range_v7(r, az_in, el_in, s_az, s_el, opts);
% 
%     fprintf('az_in=%.2f el_in=%.2f cmd_az=%.2f cmd_el=%.2f dist=%.1f str=%d hold_az=%d hold_el=%d\n', ...
%         az_in, el_in, cmd_az, cmd_el, dist_cm, strength, status.track.no_action_az, status.track.no_action_el);
% 
%     pause(0.05);
% end
%=============================================================
function [dist_cm, strength, cmd_az, cmd_el, status] = get_range_v7(r, azimuth, elevation, s_az, s_el, opts_in)

persistent tracker_state;
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
        t0 = [];
        last_t = [];
        lidar_read_tf02_v7([], default_opts(), "reset");
        status.reset = true;
        status.message = "Tracker and LiDAR states reset";
        return;
    end
end

if nargin < 5
    error('get_range_v7 requires r, azimuth, elevation, s_az, s_el.');
end

if nargin < 6
    opts_in = struct();
end
opts = merge_opts(default_opts(), opts_in);

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

[tracker_state, cmd_az, cmd_el, track_status] = tracker_step_v7( ...
    tracker_state, azimuth, elevation, t_now, opts);

% Send each servo command only when that axis is permitted.
if ~track_status.no_action_az
    writePosition(s_az, cmd_az);
end
if ~track_status.no_action_el
    writePosition(s_el, cmd_el);
end

[dist_cm, strength, lidar_status] = lidar_read_tf02_v7(r, opts);

status = struct();
status.track = track_status;
status.lidar = lidar_status;
status.options = opts;
end

function opts = default_opts()
opts = struct();

% Mapping/motion constraints
opts.lower_az = 10;      % opts.lower_az: minimum allowed mapped azimuth (deg)
opts.upper_az = 169;     % opts.upper_az: maximum allowed mapped azimuth (deg)
opts.lower_el = 5;      % opts.lower_el: minimum allowed elevation after offset (deg)
opts.upper_el = 60;      % opts.upper_el: maximum allowed elevation after offset (deg)
opts.el_offset = 7;      % opts.el_offset: elevation bias added to each input before bounds check
opts.max_move_az = 10;   % opts.max_move_az: max azimuth step per update (deg)
opts.max_move_el = 5;    % opts.max_move_el: max elevation step per update (deg)

% Prediction model
opts.window_size = 6;          % opts.window_size: number of recent samples kept for regression
opts.min_points_for_fit = 4;   % opts.min_points_for_fit: minimum samples before using regression
opts.predict_horizon_sec = 0.06; % opts.predict_horizon_sec: prediction look-ahead (s)

% Blending behavior
opts.blend_alpha = 0.70;   % opts.blend_alpha: fixed blend weight for measurement vs prediction

% UART settings
opts.serial_port = '/dev/serial0'; % opts.serial_port: UART device path on Raspberry Pi
opts.serial_baud = 115200;         % opts.serial_baud: UART baud rate
opts.serial_timeout_sec = 0.1;     % opts.serial_timeout_sec: serial read timeout (s)
opts.serial_burst_bytes = 36;      % opts.serial_burst_bytes: bytes requested per read burst
opts.serial_keep_latest_bytes = 72; % opts.serial_keep_latest_bytes: bytes to retain when dropping stale backlog
opts.serial_max_read_bytes = 360;   % opts.serial_max_read_bytes: cap for adaptive burst reads
opts.serial_max_retries = 1;       % opts.serial_max_retries: reconnect attempts per call
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
