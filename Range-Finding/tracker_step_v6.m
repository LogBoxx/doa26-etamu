% File: tracker_step_v6.m
% Purpose: Pure tracking/prediction step (no hardware I/O).

function [state, cmd_az, cmd_el, status] = tracker_step_v6(state, az_in, el_in, t_now, opts, conf_in)

% Initialize tracker state on first call.
if isempty(state)
    state.last_az = opts.lower_az;   % opts.lower_az: azimuth lower command limit (deg)
    state.last_el = opts.lower_el;   % opts.lower_el: elevation lower command limit (deg)
    state.hist_t = [];
    state.hist_az = [];
    state.hist_el = [];
    state.hist_conf = [];
end

status = struct();
status.measurement_valid = isfinite(az_in) && isfinite(el_in);
status.az_in_bounds = false;
status.el_in_bounds = false;
status.input_in_bounds = false;
status.no_action = false;
status.history_added = false;
status.used_prediction = false;
status.conf_used = conf_in;
status.blend_alpha = NaN;
status.az_pred = state.last_az;
status.el_pred = state.last_el;

if status.measurement_valid
    % Keep existing azimuth mapping convention.
    az_mapped = az_in + 90;
    % Apply configured elevation offset before limit checks.
    el_mapped = el_in + opts.el_offset; % opts.el_offset: elevation bias (deg) before bounds check

    status.az_in_bounds = (az_mapped >= opts.lower_az) && (az_mapped <= opts.upper_az); % opts.lower_az/opts.upper_az: azimuth valid window
    status.el_in_bounds = (el_mapped >= opts.lower_el) && (el_mapped <= opts.upper_el); % opts.lower_el/opts.upper_el: elevation valid window
    status.input_in_bounds = status.az_in_bounds && status.el_in_bounds;
end

% Requirement: if either channel is out-of-bounds, hold both servos and skip model update.
if ~status.input_in_bounds
    cmd_az = state.last_az;
    cmd_el = state.last_el;
    status.no_action = true;
    return;
end

% Inputs are already verified in-bounds above.
az_meas = az_mapped;
el_meas = el_mapped;

% Add only in-bounds samples to history, with confidence gating.
if (conf_in >= opts.conf_gate) || isempty(state.hist_t) % opts.conf_gate: minimum confidence to accept sample
    state.hist_t(end+1) = t_now;
    state.hist_az(end+1) = az_meas;
    state.hist_el(end+1) = el_meas;
    state.hist_conf(end+1) = conf_in;
    status.history_added = true;
end

if numel(state.hist_t) > opts.window_size % opts.window_size: rolling history size
    state.hist_t = state.hist_t(end-opts.window_size+1:end);       % opts.window_size: rolling history size
    state.hist_az = state.hist_az(end-opts.window_size+1:end);     % opts.window_size: rolling history size
    state.hist_el = state.hist_el(end-opts.window_size+1:end);     % opts.window_size: rolling history size
    state.hist_conf = state.hist_conf(end-opts.window_size+1:end); % opts.window_size: rolling history size
end

pred_az = az_meas;
pred_el = el_meas;

if numel(state.hist_t) >= opts.min_points_for_fit % opts.min_points_for_fit: minimum samples before regression
    % Weighted linear least squares (higher confidence => higher weight).
    X = [state.hist_t(:), ones(numel(state.hist_t), 1)];
    w = max(state.hist_conf(:), 1e-3);
    sw = sqrt(w);
    Xw = X .* sw;

    b_az = Xw \ (state.hist_az(:) .* sw);
    b_el = Xw \ (state.hist_el(:) .* sw);

    t_pred = t_now + opts.predict_horizon_sec; % opts.predict_horizon_sec: one-step look-ahead horizon (s)
    pred_az = [t_pred, 1] * b_az;
    pred_el = [t_pred, 1] * b_el;
    status.used_prediction = true;
end

pred_az = min(max(pred_az, opts.lower_az), opts.upper_az); % opts.lower_az/opts.upper_az: azimuth clamp for prediction
pred_el = min(max(pred_el, opts.lower_el), opts.upper_el); % opts.lower_el/opts.upper_el: elevation clamp for prediction

% Confidence-adaptive blend between raw measurement and model prediction.
blend_alpha = opts.alpha_lo + (opts.alpha_hi - opts.alpha_lo) * conf_in; % opts.alpha_lo/opts.alpha_hi: confidence blend range
blend_alpha = min(max(blend_alpha, opts.alpha_lo), opts.alpha_hi);        % opts.alpha_lo/opts.alpha_hi: enforce blend bounds
status.blend_alpha = blend_alpha;

az_target = blend_alpha * az_meas + (1 - blend_alpha) * pred_az;
el_target = blend_alpha * el_meas + (1 - blend_alpha) * pred_el;

% Apply per-axis slew limits.
cmd_az = az_target;
if abs(cmd_az - state.last_az) > opts.max_move_az % opts.max_move_az: max azimuth change per update (deg)
    if (cmd_az - state.last_az) > 0
        cmd_az = state.last_az + opts.max_move_az; % opts.max_move_az: max azimuth change per update (deg)
    else
        cmd_az = state.last_az - opts.max_move_az; % opts.max_move_az: max azimuth change per update (deg)
    end
end

cmd_el = el_target;
if abs(cmd_el - state.last_el) > opts.max_move_el % opts.max_move_el: max elevation change per update (deg)
    if (cmd_el - state.last_el) > 0
        cmd_el = state.last_el + opts.max_move_el; % opts.max_move_el: max elevation change per update (deg)
    else
        cmd_el = state.last_el - opts.max_move_el; % opts.max_move_el: max elevation change per update (deg)
    end
end

state.last_az = cmd_az;
state.last_el = cmd_el;

status.az_pred = pred_az;
status.el_pred = pred_el;
end
