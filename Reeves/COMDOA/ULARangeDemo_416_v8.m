% Script    ULARangeDemo_416_v8
% Purpose   V8 real-time globe plotting during estimation loop
% Notes     (1) Implements FB averaging
%           (2) Stores estimated az/el/dist each trial
%           (3) Updates globe/text in real time (no post-loop replay required)
% Author    Parker Reeves
% Date      04/16/2026

clear; clf
u = udpport("IPV4", "LocalPort", 5005);
rpi = raspi('169.254.67.1', 'analog', 'analog');

if ~exist('s_az', 'var')
    s_az = servo(rpi, 13, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
end
if ~exist('s_el', 'var')
    s_el = servo(rpi, 12, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
end
point_servo_v2('reset');

x_direct = adi.FMComms5.Rx('uri','ip:192.168.1.101');
x_direct.EnabledChannels = [1 2 3 4];
x_direct.SamplesPerFrame = 2^15;

M = 4;                   % Number of array elements
d = 0.5;                 % Element spacing (in wavelengths)
c = 3e8; f = 2.4e9;      % Speed of light and operating frequency
lambda = c/f;            % Wavelength (m)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters

theta_scan = -90:1:90; % Angle search grid (degrees)
num_trials = 500;

% Stored estimation history
m_AZ = nan(1, num_trials);
m_EL = zeros(1, num_trials);      % ULA range demo is azimuth-only
m_DIST_M = nan(1, num_trials);
m_DIST_CM = nan(1, num_trials);

Navg = 12;
est_hist = nan(Navg,1);

% ============================ GLOBE PLOT (from v67) ============================ %
trail_len = 10;
trail_xyz = nan(trail_len, 3);
trail_idx = 0;
trail_count = 0;
rmax_cm = 300;

fig_globe = figure;
ax_globe = axes('Parent', fig_globe);
hold(ax_globe, 'on');
grid(ax_globe, 'on');
view(ax_globe, 3);
axis(ax_globe, 'equal');
xlim(ax_globe, [0 rmax_cm]);
ylim(ax_globe, [-rmax_cm rmax_cm]);
zlim(ax_globe, [-rmax_cm/4 rmax_cm]);
xlabel(ax_globe, 'X (cm)');
ylabel(ax_globe, 'Y (cm)');
zlabel(ax_globe, 'Z (cm)');
title(ax_globe, 'Option 3: Radar Globe View (Az/El Limited to +/-90 deg)');

[az_grid, el_grid] = meshgrid(-90:3:90, -30:3:90);
xs = rmax_cm .* cosd(el_grid) .* cosd(az_grid);
ys = rmax_cm .* cosd(el_grid) .* sind(az_grid);
zs = rmax_cm .* sind(el_grid);
surf(ax_globe, xs, ys, zs, ...
     'FaceAlpha', 0.2, 'EdgeAlpha', 0.12, ...
     'FaceColor', [0.15 0.55 0.85], 'EdgeColor', [0.15 0.55 0.85]);

floor_el_deg = 0;
[az_floor, r_floor] = meshgrid(-90:3:90, linspace(0, rmax_cm, 40));
xf = r_floor .* cosd(floor_el_deg) .* cosd(az_floor);
yf = r_floor .* cosd(floor_el_deg) .* sind(az_floor);
zf = r_floor .* sind(floor_el_deg);
surf(ax_globe, xf, yf, zf, ...
     'FaceAlpha', 0.2, 'EdgeAlpha', 0.10, ...
     'FaceColor', [0.2 0.8 0.35], 'EdgeColor', [0.2 0.8 0.35]);

t_ref = linspace(-90, 90, 240);
plot3(ax_globe, rmax_cm*cosd(t_ref), rmax_cm*sind(t_ref), zeros(size(t_ref)), ...
      '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
plot3(ax_globe, rmax_cm*cosd(t_ref), zeros(size(t_ref)), rmax_cm*sind(t_ref), ...
      '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);

h_globe_trail = plot3(ax_globe, nan, nan, nan, '-', 'LineWidth', 2, ...
                      'Color', [0.95 0.55 0.15]);
h_globe_pt = plot3(ax_globe, nan, nan, nan, 'o', 'MarkerSize', 8, ...
                   'MarkerFaceColor', [0.95 0.2 0.2], ...
                   'MarkerEdgeColor', [0.2 0.2 0.2]);
h_globe_ray = plot3(ax_globe, [0 nan], [0 nan], [0 nan], '-', ...
                    'LineWidth', 1.2, 'Color', [0.95 0.2 0.2]);

h_status_dist = annotation(fig_globe, 'textbox', [0.02 0.1 0.34 0.05], ...
                           'String', 'Estimated Distance: -- cm', ...
                           'FitBoxToText', 'off', ...
                           'HorizontalAlignment', 'left', ...
                           'VerticalAlignment', 'bottom', ...
                           'LineStyle', 'none', ...
                           'FontSize', 15, ...
                           'Color', [0.85 0.1 0.1]);

h_status_angles = annotation(fig_globe, 'textbox', [0.02 0.02 0.34 0.06], ...
                             'String', sprintf(['Estimated Azimuth:  -- deg\n', ...
                                                'Estimated Elevation: -- deg']), ...
                             'FitBoxToText', 'off', ...
                             'HorizontalAlignment', 'left', ...
                             'VerticalAlignment', 'bottom', ...
                             'LineStyle', 'none', ...
                             'FontSize', 15, ...
                             'Color', [0.2 0.8 0.35]);

% ============================ ESTIMATION LOOP =========================== %
for o = 1:num_trials
    X = fliplr(x_direct());
    J = fliplr(eye(M));
    En = MusicAlg_v2(X, J);
    Pmusic = zeros(size(theta_scan));

    for t = 1:length(theta_scan)
        a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));
        Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);
    end

    [~, idx_peak] = max(Pmusic);
    est_DOAs_x = theta_scan(idx_peak);

    est_hist = [est_hist(2:end,:); est_DOAs_x(:).'];
    est_avg  = mean(est_hist, 1, 'omitnan');

    [cmd_az, cmd_el, status] = point_servo_v2(rpi, est_avg, 25, s_az, s_el); %
    dist_m = read_range(u, 4);
    dist_cm = dist_m * 100;

    est_AZ = est_avg;
    est_EL = 0;

    % Store trial data.
    m_AZ(o) = est_AZ;
    m_EL(o) = est_EL;
    m_DIST_M(o) = dist_m;
    m_DIST_CM(o) = dist_cm;

    % Real-time globe update.
    x_cm = dist_cm * cosd(est_EL) * cosd(est_AZ);
    y_cm = dist_cm * cosd(est_EL) * sind(est_AZ);
    z_cm = dist_cm * sind(est_EL);

    trail_idx = mod(trail_idx, trail_len) + 1;
    trail_xyz(trail_idx, :) = [x_cm, y_cm, z_cm];
    trail_count = min(trail_count + 1, trail_len);

    if trail_count < trail_len
        trail_order = 1:trail_count;
    else
        trail_order = [trail_idx+1:trail_len, 1:trail_idx];
    end

    set(h_globe_trail, 'XData', trail_xyz(trail_order,1), ...
                       'YData', trail_xyz(trail_order,2), ...
                       'ZData', trail_xyz(trail_order,3));
    set(h_globe_pt, 'XData', x_cm, 'YData', y_cm, 'ZData', z_cm);
    set(h_globe_ray, 'XData', [0 x_cm], 'YData', [0 y_cm], 'ZData', [0 z_cm]);

    if dist_cm < 500
        dist_color = [0.1 0.65 0.1];
    else
        dist_color = [0.85 0.1 0.1];
    end

    set(h_status_dist, 'String', sprintf('Estimated Distance: %.1f cm', dist_cm), ...
                       'Color', dist_color);
    set(h_status_angles, 'String', sprintf(['Estimated Azimuth:  %.2f deg\n', ...
                                            'Estimated Elevation: %.2f deg'], ...
                                            est_AZ, est_EL));

    drawnow limitrate;
    fprintf('Trial %d | Estimated DOA: %.2f deg | Distance: %.2f cm\n', o, est_AZ, dist_cm);
end

% ========================= TIME-SERIES SUMMARY PLOT ========================= %
figure;

subplot(2,1,1)
plot(m_AZ, 'rx', 'MarkerSize', 8, 'LineWidth', 1.5)
xlim([0, num_trials])
ylim([-90, 90])
title('ULA SVD-QR MUSIC')
xlabel('Trial Number')
ylabel('Azimuth Estimation')

subplot(2,1,2)
plot(m_DIST_M, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5)
xlim([0, num_trials])
ylim([0, 10])
title('Distance')
xlabel('Trial Number')
ylabel('Distance (m)')
