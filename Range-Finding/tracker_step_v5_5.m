% File: tracker_step_v5_5.m
% Purpose: Pure tracking/prediction step (no hardware I/O).

function [state, cmd_az, cmd_el, status] = tracker_step_v5_5(state, az_in, el_in, t_now, opts, conf_in)

% Initialize state on first call.
if isempty(state)
    state.last_az = opts.lower_az;
    state.last_el = opts.lower_el;
    state.hist_t = [];
    state.hist_az = [];
    state.hist_el = [];
    state.hist_conf = [];
end

status = struct();
% Treat NaN/Inf estimator outputs as invalid measurements.
status.measurement_valid = isfinite(az_in) && isfinite(el_in);
status.hold_azimuth = false;
status.used_prediction = false;
status.conf_used = conf_in;
status.blend_alpha = NaN;
status.az_pred = NaN;
status.el_pred = NaN;

if ~status.measurement_valid
    % Fall back to last command when estimator values are invalid.
    mapped = state.last_az;
    el_meas = state.last_el;
    status.used_prediction = true;
else
    % Keep existing azimuth mapping convention.
    mapped = az_in + 90;
    if mapped > opts.upper_az
        mapped = opts.upper_az;
        status.hold_azimuth = true;
    elseif mapped < opts.lower_az
        mapped = opts.lower_az;
        status.hold_azimuth = true;
    end

    % Apply configured elevation offset before clamping.
    el_in = el_in + opts.el_offset;
    if el_in > opts.upper_el
        el_meas = opts.upper_el;
    elseif el_in < opts.lower_el
        el_meas = opts.lower_el;
    else
        el_meas = el_in;
    end
end

az_meas = mapped;

% Build history from mapped/clamped values, but gate low-confidence samples.
if (conf_in >= opts.conf_gate) || isempty(state.hist_t)
    state.hist_t(end+1) = t_now;
    state.hist_az(end+1) = az_meas;
    state.hist_el(end+1) = el_meas;
    state.hist_conf(end+1) = conf_in;
end

if numel(state.hist_t) > opts.window_size
    state.hist_t = state.hist_t(end-opts.window_size+1:end);
    state.hist_az = state.hist_az(end-opts.window_size+1:end);
    state.hist_el = state.hist_el(end-opts.window_size+1:end);
    state.hist_conf = state.hist_conf(end-opts.window_size+1:end);
end

pred_az = az_meas;
pred_el = el_meas;

if numel(state.hist_t) >= opts.min_points_for_fit
    % Weighted linear least squares (higher confidence => higher weight).
    X = [state.hist_t(:), ones(numel(state.hist_t), 1)];
    w = max(state.hist_conf(:), 1e-3);
    sw = sqrt(w);
    Xw = X .* sw;

    b_az = Xw \ (state.hist_az(:) .* sw);
    b_el = Xw \ (state.hist_el(:) .* sw);

    t_pred = t_now + opts.predict_horizon_sec;
    pred_az = [t_pred, 1] * b_az;
    pred_el = [t_pred, 1] * b_el;
    status.used_prediction = true;
end

pred_az = min(max(pred_az, opts.lower_az), opts.upper_az);
pred_el = min(max(pred_el, opts.lower_el), opts.upper_el);

% Confidence-adaptive blend between raw measurement and model prediction.
blend_alpha = opts.alpha_lo + (opts.alpha_hi - opts.alpha_lo) * conf_in;
blend_alpha = min(max(blend_alpha, opts.alpha_lo), opts.alpha_hi);
status.blend_alpha = blend_alpha;

az_target = blend_alpha * az_meas + (1 - blend_alpha) * pred_az;
el_target = blend_alpha * el_meas + (1 - blend_alpha) * pred_el;

cmd_az = az_target;
if status.hold_azimuth
    cmd_az = state.last_az;
else
    % Apply azimuth slew-rate limit.
    if abs(cmd_az - state.last_az) > opts.max_move_az
        if (cmd_az - state.last_az) > 0
            cmd_az = state.last_az + opts.max_move_az;
        else
            cmd_az = state.last_az - opts.max_move_az;
        end
    end
end

cmd_el = el_target;
% Apply elevation slew-rate limit.
if abs(cmd_el - state.last_el) > opts.max_move_el
    if (cmd_el - state.last_el) > 0
        cmd_el = state.last_el + opts.max_move_el;
    else
        cmd_el = state.last_el - opts.max_move_el;
    end
end

state.last_az = cmd_az;
state.last_el = cmd_el;

status.az_pred = pred_az;
status.el_pred = pred_el;
end
