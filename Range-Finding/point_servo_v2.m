% File: point_servo_v2.m
% Purpose: Move servos
% Author: Alden Edwards
% Usage:[cmd_az, cmd_el, status] = point_servo_v2(r, azimuth, elevation, s_az, s_el, opts_in)
%       point_servo('reset')
%
% Date: 4/15/26

function [cmd_az, cmd_el, status] = point_servo_v2(r, azimuth, elevation, s_az, s_el, opts_in)

persistent tracker_state;
persistent t0;
persistent last_t;
cmd_az = NaN;
cmd_el = NaN;
status = struct();

if nargin > 0 && strcmpi(r, 'reset')
    tracker_state = [];
    t0 = [];
    last_t = [];
    status = struct('reset', true, 'message', "Tracker state reset");
    return;
end

narginchk(5,6);
if nargin < 6, opts_in = struct(); end

opts = merge_opts(default_opts(), opts_in);

if isempty(t0) % Initialize time base
    t0 = tic;
    last_t = 0;
end

t_now = max(toc(t0), last_t + 1e-3); % Monotonic time
last_t = t_now;

% Tracker Step
[tracker_state, cmd_az, cmd_el, track_status] = tracker_step_v8( ...
    tracker_state, azimuth, elevation, t_now, opts);

% Send each servo command only when that axis is permitted.
if ~track_status.no_action_az
    writePosition(s_az, cmd_az);
end
if ~track_status.no_action_el
    writePosition(s_el, cmd_el);
end

status.track = track_status;
status.options = opts;
end

function opts = default_opts()
opts = struct();

%============ Mapping/motion constraints ==================================
opts.lower_az = 10;      % minimum allowed mapped azimuth (deg)
opts.upper_az = 169;     % maximum allowed mapped azimuth (deg)
opts.lower_el = 15;      % minimum allowed elevation after offset (deg)
opts.upper_el = 60;      % maximum allowed elevation after offset (deg)
opts.el_offset = 0;      % elevation bias added to each input before bounds check
opts.max_move_az = 10;   % max azimuth step per update (deg)
opts.max_move_el = 5;    % max elevation step per update (deg)

%========Prediction model
opts.window_size = 6;             % number of recent samples kept for regression
opts.min_points_for_fit = 4;      % minimum samples before using regression
opts.predict_horizon_sec = 0.06;  % prediction look-ahead (s)

%========Blending behavior
opts.blend_alpha = 0.70;  % α · measured + (1−α) · predicted = cmd_

opts.bytes_per_sample = 4;  % sample length for read_range(typecast single)
end
%==========================================================================

function out = merge_opts(base, override)
    if isempty(override)
        out = base;
    else
        out = base;
        f = fieldnames(override);
        [out.(f{:})] = deal(override.(f{:}));
    end
end
