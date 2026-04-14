% File: point_servo_test.m
% Purpose: Hardware demo for point_servo (tracking + prediction + range).

clear all;
clc;
u = udpport("IPV4", "LocalPort", 5005);
rpi = raspi('169.254.67.1', 'analog', 'analog');

if ~exist('s_az', 'var')
    s_az = servo(rpi, 13, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
end
if ~exist('s_el', 'var')
    s_el = servo(rpi, 12, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
end

% Optional tuning overrides (leave empty struct to use defaults in point_servo)
opts = struct(); % opts: override any default field from point_servo.
opts.bytes_per_sample = 4; % bytes consumed by read_range packet
% Example override:
% opts.max_move_az = 8; opts.max_move_el = 6;
% opts.el_offset = -2;

% Reset persistent state between runs.
point_servo('reset');

N = 50;
dt = 0.05;
noise_az = 2.0;
noise_el = 2.0;

az_in_log = zeros(1, N);
el_in_log = zeros(1, N);
az_src_unclamped_log = zeros(1, N);
el_src_unclamped_log = zeros(1, N);
az_src_clamped_log = zeros(1, N);
el_src_clamped_log = zeros(1, N);
az_cmd_log = zeros(1, N);
el_cmd_log = zeros(1, N);
dist_log = NaN(1, N);
hold_az_log = false(1, N);
hold_el_log = false(1, N);

fprintf('Starting point_servo demo for %d steps...\n', N);
fprintf('Step | az_in el_in | cmd_az cmd_el | dist_m hold_az hold_el\n');

for k = 1:N
    % Demo source estimate (replace with your estimator stream).
    az_in = 65 * sin(2*pi*k/80) + 8 * sin(2*pi*k/23) + noise_az * randn();
    el_in = 35 * sin(2*pi*k/90 + 0.5) + 30 + noise_el * randn();

    [dist_m, cmd_az, cmd_el, status] = point_servo(u, az_in, el_in, s_az, s_el, opts);

    az_in_log(k) = az_in;
    el_in_log(k) = el_in;
    az_src_unclamped_log(k) = az_in + 90;
    el_src_unclamped_log(k) = el_in + status.options.el_offset;
    az_src_clamped_log(k) = min(max(az_in + 90, status.options.lower_az), status.options.upper_az);
    el_src_clamped_log(k) = min(max(el_in + status.options.el_offset, status.options.lower_el), status.options.upper_el);
    az_cmd_log(k) = cmd_az;
    el_cmd_log(k) = cmd_el;
    dist_log(k) = dist_m;
    hold_az_log(k) = status.track.no_action_az;
    hold_el_log(k) = status.track.no_action_el;

    fprintf('%4d | %5.1f %5.1f | %6.1f %6.1f | %6.3f %7d %7d\n', ...
        k, az_in, el_in, cmd_az, cmd_el, dist_m, status.track.no_action_az, status.track.no_action_el);
    pause(dt);
end

% Plot source request, clamped target, servo command, and range output.
figure('Color', 'w', 'Name', 'point\_servo Source vs Servo');
t = 1:N;

subplot(3,1,1);
plot(t, az_src_unclamped_log, 'Color', [0.65 0.65 0.65], 'LineWidth', 1.1); hold on;
plot(t, az_src_clamped_log, 'k-', 'LineWidth', 1.3);
plot(t, az_cmd_log, 'r-', 'LineWidth', 1.4);
yline(status.options.lower_az, 'k:');
yline(status.options.upper_az, 'k:');
grid on;
ylabel('Azimuth (deg)');
title('Azimuth: Unclamped Source vs Clamped Target vs Servo');
legend('Source (mapped, unclamped)', 'Source target (clamped)', 'Servo', 'Location', 'northwest');

subplot(3,1,2);
plot(t, el_src_unclamped_log, 'Color', [0.65 0.65 0.65], 'LineWidth', 1.1); hold on;
plot(t, el_src_clamped_log, 'k-', 'LineWidth', 1.3);
plot(t, el_cmd_log, 'r-', 'LineWidth', 1.4);
yline(status.options.lower_el, 'k:');
yline(status.options.upper_el, 'k:');
grid on;
ylabel('Elevation (deg)');
title('Elevation: Unclamped Source vs Clamped Target vs Servo');
legend('Source (unclamped)', 'Source target (clamped)', 'Servo', 'Location', 'northwest');

subplot(3,1,3);
plot(t, dist_log, 'b-', 'LineWidth', 1.3);
grid on;
xlabel('Step');
ylabel('Distance (m)');
title('Range Output');
