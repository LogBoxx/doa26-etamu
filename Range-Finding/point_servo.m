% File: point_servo.m
% Purpose: Tracker wrapper for servo pointing without LiDAR I/O.
%
% Usage:
%   [cmd_az, cmd_el, status] = point_servo(r, az_in, el_in, s_az, s_el)
%   [cmd_az, cmd_el, status] = point_servo(r, az_in, el_in, s_az, s_el, opts)
%   point_servo('reset')
%
% Inputs are kept identical to get_range_v7 for drop-in compatibility.

function [cmd_az, cmd_el, status] = point_servo(r, azimuth, elevation, s_az, s_el, opts_in)

persistent tracker_state;
persistent t0;
persistent last_t;

cmd_az = NaN;
cmd_el = NaN;
status = struct();

if nargin >= 1 && (ischar(r) || isstring(r))
    cmd = lower(string(r));
    if cmd == "reset"
        tracker_state = [];
        t0 = [];
        last_t = [];
        status.reset = true;
        status.message = "Tracker state reset";
        return;
    end
end

if nargin < 5
    error('point_servo requires r, azimuth, elevation, s_az, s_el.');
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

status = struct();
status.track = track_status;
status.options = opts;
end

function opts = default_opts()
opts = struct();

% Mapping/motion constraints
opts.lower_az = 10;      % opts.lower_az: minimum allowed mapped azimuth (deg)
opts.upper_az = 169;     % opts.upper_az: maximum allowed mapped azimuth (deg)
opts.lower_el = 15;      % opts.lower_el: minimum allowed elevation after offset (deg)
opts.upper_el = 60;      % opts.upper_el: maximum allowed elevation after offset (deg)
opts.el_offset = 0;      % opts.el_offset: elevation bias added to each input before bounds check
opts.max_move_az = 10;   % opts.max_move_az: max azimuth step per update (deg)
opts.max_move_el = 5;    % opts.max_move_el: max elevation step per update (deg)

% Prediction model
opts.window_size = 6;             % opts.window_size: number of recent samples kept for regression
opts.min_points_for_fit = 4;      % opts.min_points_for_fit: minimum samples before using regression
opts.predict_horizon_sec = 0.06;  % opts.predict_horizon_sec: prediction look-ahead (s)

% Blending behavior
opts.blend_alpha = 0.70;          % opts.blend_alpha: measurement blend weight
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
