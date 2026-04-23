% File: tracker_step_v8.m
% Purpose: Servo mapping and tracking/prediction (no hardware I/O).
% Date: 4/15/26
% Author: Alden Edwards

function [state, cmd_az, cmd_el, status] = tracker_step_v8(state, az_in, el_in, t_now, opts)



%===============Initialize all Variables===================================

if isempty(state) % Initialize tracker state on first call.
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

%================Mapping Array Estimates to Servo posistions================
if status.measurement_valid_az
    az_mapped = az_in + 90;
    status.az_in_bounds = (az_mapped >= opts.lower_az) && (az_mapped <= opts.upper_az); % opts.lower_az/opts.upper_az: azimuth valid window
end
if status.measurement_valid_el
    el_mapped = el_in + opts.el_offset; % Apply opts.el_offset before limit checks.
    status.el_in_bounds = (el_mapped >= opts.lower_el) && (el_mapped <= opts.upper_el); % opts.lower_el/opts.upper_el: elevation valid window
end

%================= Linear Regression for DoA prediction====================

blend_alpha = opts.blend_alpha; % opts.blend_alpha: measurement blend weight
status.blend_alpha = blend_alpha;
cmd_az = state.last_az; % Default prev. unless input is valid and in bounds.
cmd_el = state.last_el;

%========Azimuth path======
% The tracker Capstoneumes elevation is moving at a roughly constant angular velocity.
% Given a history of (time, azimuth) samples, it fits the best straight line through them
% then reads off where that line lands slightly in the future.
% The line model is: azimuth = slope × time + intercept
% Or in vector form: az = b[0]·t + b[1]·1

if status.az_in_bounds 
    state.hist_t_az(end+1) = t_now;
    state.hist_az(end+1) = az_mapped;
    status.history_added_az = true;

    if numel(state.hist_t_az) > opts.window_size % opts.window_size: rolling history size
        state.hist_t_az = state.hist_t_az(end-opts.window_size+1:end); 
        state.hist_az = state.hist_az(end-opts.window_size+1:end);       
    end

    pred_az = az_mapped;
    if numel(state.hist_t_az) >= opts.min_points_for_fit % check for minimum samples before regression
        Xaz = [state.hist_t_az(:), ones(numel(state.hist_t_az), 1)]; %each row is a sample , Xaz = [t 1]
        baz = Xaz \ state.hist_az(:); % X · b = y -> minimize  ‖X·b − y‖²
        t_pred = t_now + opts.predict_horizon_sec; % opts.predict_horizon_sec: look-ahead horizon (s)
        pred_az = [t_pred, 1] * baz; % y = mx + b
        status.used_prediction_az = true;
    end

    pred_az = min(max(pred_az, opts.lower_az), opts.upper_az); % opts.lower_az/opts.upper_az: azimuth clamp for prediction
    az_target = blend_alpha * az_mapped + (1 - blend_alpha) * pred_az; % α · measured + (1−α) · predicted
    cmd_az = az_target;

    if abs(cmd_az - state.last_az) > opts.max_move_az
        if (cmd_az - state.last_az) > 0
            cmd_az = state.last_az + opts.max_move_az;
        else
            cmd_az = state.last_az - opts.max_move_az;
        end
    end
    state.last_az = cmd_az;
    status.az_pred = pred_az;
    status.no_action_az = false;
end

%========Elevation path(Same logic as Azimuth)======

if status.el_in_bounds
    state.hist_t_el(end+1) = t_now;
    state.hist_el(end+1) = el_mapped;
    status.history_added_el = true;

    if numel(state.hist_t_el) > opts.window_size 
        state.hist_t_el = state.hist_t_el(end-opts.window_size+1:end);         
        state.hist_el = state.hist_el(end-opts.window_size+1:end);             
    end

    pred_el = el_mapped;
    if numel(state.hist_t_el) >= opts.min_points_for_fit 
        Xel = [state.hist_t_el(:), ones(numel(state.hist_t_el), 1)];
        bel = Xel \ state.hist_el(:);
        t_pred = t_now + opts.predict_horizon_sec; 
        pred_el = [t_pred, 1] * bel;
        status.used_prediction_el = true;
    end
    pred_el = min(max(pred_el, opts.lower_el), opts.upper_el);
    el_target = blend_alpha * el_mapped + (1 - blend_alpha) * pred_el;
    cmd_el = el_target;
    if abs(cmd_el - state.last_el) > opts.max_move_el
        if (cmd_el - state.last_el) > 0
            cmd_el = state.last_el + opts.max_move_el;
        else
            cmd_el = state.last_el - opts.max_move_el;
        end
    end
    state.last_el = cmd_el;
    status.el_pred = pred_el;
    status.no_action_el = false;
end

status.no_action = status.no_action_az && status.no_action_el;
status.used_prediction = status.used_prediction_az || status.used_prediction_el;
end