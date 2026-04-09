% File: tracker_step_v7.m
% Purpose: Pure tracking/prediction step (no hardware I/O).

function [state, cmd_az, cmd_el, status] = tracker_step_v7(state, az_in, el_in, t_now, opts)

% Initialize tracker state on first call.
if isempty(state)
    state.last_az = opts.lower_az;   % opts.lower_az: azimuth lower command limit (deg)
    state.last_el = opts.lower_el;   % opts.lower_el: elevation lower command limit (deg)
    % Keep axis-specific histories so one bad axis does not block the other.
    state.hist_t_az = [];
    state.hist_az = [];
    state.hist_t_el = [];
    state.hist_el = [];
end

status = struct();
status.measurement_valid_az = isfinite(az_in);
status.measurement_valid_el = isfinite(el_in);
status.az_in_bounds = false;
status.el_in_bounds = false;
status.no_action_az = true;
status.no_action_el = true;
status.no_action = true;
status.history_added_az = false;
status.history_added_el = false;
status.used_prediction_az = false;
status.used_prediction_el = false;
status.used_prediction = false;
status.blend_alpha = NaN;
status.az_pred = state.last_az;
status.el_pred = state.last_el;

az_mapped = NaN;
el_mapped = NaN;
if status.measurement_valid_az
    % Keep existing azimuth mapping convention.
    az_mapped = az_in + 90;
    status.az_in_bounds = (az_mapped >= opts.lower_az) && (az_mapped <= opts.upper_az); % opts.lower_az/opts.upper_az: azimuth valid window
end
if status.measurement_valid_el
    % Apply configured elevation offset before limit checks.
    el_mapped = el_in + opts.el_offset; % opts.el_offset: elevation bias (deg) before bounds check
    status.el_in_bounds = (el_mapped >= opts.lower_el) && (el_mapped <= opts.upper_el); % opts.lower_el/opts.upper_el: elevation valid window
end

% Fixed blend between raw measurement and model prediction.
blend_alpha = min(max(opts.blend_alpha, 0), 1); % opts.blend_alpha: measurement blend weight
status.blend_alpha = blend_alpha;

% Default to no movement unless axis input is valid and in bounds.
cmd_az = state.last_az;
cmd_el = state.last_el;

% Azimuth path (independent from elevation path).
if status.az_in_bounds
    state.hist_t_az(end+1) = t_now;
    state.hist_az(end+1) = az_mapped;
    status.history_added_az = true;

    if numel(state.hist_t_az) > opts.window_size % opts.window_size: rolling history size
        state.hist_t_az = state.hist_t_az(end-opts.window_size+1:end);         % opts.window_size: rolling history size
        state.hist_az = state.hist_az(end-opts.window_size+1:end);             % opts.window_size: rolling history size
    end

    pred_az = az_mapped;
    if numel(state.hist_t_az) >= opts.min_points_for_fit % opts.min_points_for_fit: minimum samples before regression
        Xaz = [state.hist_t_az(:), ones(numel(state.hist_t_az), 1)];
        baz = Xaz \ state.hist_az(:);
        t_pred = t_now + opts.predict_horizon_sec; % opts.predict_horizon_sec: one-step look-ahead horizon (s)
        pred_az = [t_pred, 1] * baz;
        status.used_prediction_az = true;
    end
    pred_az = min(max(pred_az, opts.lower_az), opts.upper_az); % opts.lower_az/opts.upper_az: azimuth clamp for prediction
    az_target = blend_alpha * az_mapped + (1 - blend_alpha) * pred_az;
    cmd_az = az_target;
    if abs(cmd_az - state.last_az) > opts.max_move_az % opts.max_move_az: max azimuth change per update (deg)
        if (cmd_az - state.last_az) > 0
            cmd_az = state.last_az + opts.max_move_az; % opts.max_move_az: max azimuth change per update (deg)
        else
            cmd_az = state.last_az - opts.max_move_az; % opts.max_move_az: max azimuth change per update (deg)
        end
    end
    state.last_az = cmd_az;
    status.az_pred = pred_az;
    status.no_action_az = false;
end

% Elevation path (independent from azimuth path).
if status.el_in_bounds
    state.hist_t_el(end+1) = t_now;
    state.hist_el(end+1) = el_mapped;
    status.history_added_el = true;

    if numel(state.hist_t_el) > opts.window_size % opts.window_size: rolling history size
        state.hist_t_el = state.hist_t_el(end-opts.window_size+1:end);         % opts.window_size: rolling history size
        state.hist_el = state.hist_el(end-opts.window_size+1:end);             % opts.window_size: rolling history size
    end

    pred_el = el_mapped;
    if numel(state.hist_t_el) >= opts.min_points_for_fit % opts.min_points_for_fit: minimum samples before regression
        Xel = [state.hist_t_el(:), ones(numel(state.hist_t_el), 1)];
        bel = Xel \ state.hist_el(:);
        t_pred = t_now + opts.predict_horizon_sec; % opts.predict_horizon_sec: one-step look-ahead horizon (s)
        pred_el = [t_pred, 1] * bel;
        status.used_prediction_el = true;
    end
    pred_el = min(max(pred_el, opts.lower_el), opts.upper_el); % opts.lower_el/opts.upper_el: elevation clamp for prediction
    el_target = blend_alpha * el_mapped + (1 - blend_alpha) * pred_el;
    cmd_el = el_target;
    if abs(cmd_el - state.last_el) > opts.max_move_el % opts.max_move_el: max elevation change per update (deg)
        if (cmd_el - state.last_el) > 0
            cmd_el = state.last_el + opts.max_move_el; % opts.max_move_el: max elevation change per update (deg)
        else
            cmd_el = state.last_el - opts.max_move_el; % opts.max_move_el: max elevation change per update (deg)
        end
    end
    state.last_el = cmd_el;
    status.el_pred = pred_el;
    status.no_action_el = false;
end

status.no_action = status.no_action_az && status.no_action_el;
status.used_prediction = status.used_prediction_az || status.used_prediction_el;
end
