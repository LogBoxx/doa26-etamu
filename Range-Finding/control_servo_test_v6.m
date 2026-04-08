% File: control_servo_test_v6.m
% Purpose: Hardware demo for get_range_v6 with plotting and status display.

clear all;
clc;

r = raspi('169.254.52.8', 'analog', 'analog');

if ~exist('s_az', 'var')
    s_az = servo(r, 13, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
end
if ~exist('s_el', 'var')
    s_el = servo(r, 12, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
end

% Optional tuning overrides (leave empty struct to use defaults in get_range_v6)
opts = struct(); % opts: override any default field from get_range_v6.
% Example override:
% opts.max_move_az = 8; opts.max_move_el = 6;
% opts.el_offset = -2; % degrees added to each elevation input before clamp.

% Reset persistent state between runs.
get_range_v6('reset');

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
strength_log = NaN(1, N);
conf_log = NaN(1, N);
frame_ok_log = false(1, N);
lower_az = 10;
upper_az = 169;
lower_el = 15;
upper_el = 70;

fprintf('Starting v6 tracking demo for %d steps...\n', N);
fprintf('Step | az_in el_in | cmd_az cmd_el | dist strength conf frame_ok\n');

for k = 1:N
    % Demo source estimate (replace with your estimator stream).
    az_in = 65 * sin(2*pi*k/80) + 8 * sin(2*pi*k/23) + noise_az * randn();
    el_in = 35 * sin(2*pi*k/90 + 0.5) + 30 + noise_el * randn();

    [dist_cm, strength, cmd_az, cmd_el, status] = get_range_v6(r, az_in, el_in, s_az, s_el, opts);

    az_in_log(k) = az_in;
    el_in_log(k) = el_in;
    az_src_unclamped_log(k) = az_in + 90;
    el_src_unclamped_log(k) = el_in + status.options.el_offset;
    az_src_clamped_log(k) = min(max(az_in + 90, lower_az), upper_az);
    el_src_clamped_log(k) = min(max(el_in + status.options.el_offset, lower_el), upper_el);
    az_cmd_log(k) = cmd_az;
    el_cmd_log(k) = cmd_el;
    dist_log(k) = dist_cm;
    strength_log(k) = strength;
    conf_log(k) = status.confidence_next;
    frame_ok_log(k) = status.lidar.frame_ok;

    fprintf('%4d | %5.1f %5.1f | %6.1f %6.1f | %6.1f %8.0f %4.2f %d\n', ...
        k, az_in, el_in, cmd_az, cmd_el, dist_cm, strength, status.confidence_next, status.lidar.frame_ok);
    pause(dt);
end

% Plot source request, clamped target, servo command, and LiDAR outputs.
figure('Color', 'w', 'Name', 'v6 Source vs Servo + LiDAR');
t = 1:N;

subplot(3,1,1);
plot(t, az_src_unclamped_log, 'Color', [0.65 0.65 0.65], 'LineWidth', 1.1); hold on;
plot(t, az_src_clamped_log, 'k-', 'LineWidth', 1.3);
plot(t, az_cmd_log, 'r-', 'LineWidth', 1.4);
yline(lower_az, 'k:');
yline(upper_az, 'k:');
grid on;
ylabel('Azimuth (deg)');
title('Azimuth: Unclamped Source vs Clamped Target vs Servo');
legend('Source (mapped, unclamped)', 'Source target (clamped)', 'Servo', 'Location', 'northwest');

subplot(3,1,2);
plot(t, el_src_unclamped_log, 'Color', [0.65 0.65 0.65], 'LineWidth', 1.1); hold on;
plot(t, el_src_clamped_log, 'k-', 'LineWidth', 1.3);
plot(t, el_cmd_log, 'r-', 'LineWidth', 1.4);
yline(lower_el, 'k:');
yline(upper_el, 'k:');
grid on;
ylabel('Elevation (deg)');
title('Elevation: Unclamped Source vs Clamped Target vs Servo');
legend('Source (unclamped)', 'Source target (clamped)', 'Servo', 'Location', 'northwest');

subplot(3,1,3);
yyaxis left;
plot(t, dist_log, 'b-', 'LineWidth', 1.3);
ylabel('Distance (cm)');
yyaxis right;
plot(t, strength_log, 'g-', 'LineWidth', 1.3); hold on;
plot(t, conf_log, 'm--', 'LineWidth', 1.1);
ylabel('Strength / Confidence');
grid on;
xlabel('Step');
title('LiDAR Distance, Strength, and Confidence');
legend('Distance', 'Strength', 'Confidence', 'Location', 'northwest');
