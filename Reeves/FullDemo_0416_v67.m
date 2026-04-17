% ======================================================================= %
% Script:   FullDemo_0416_v67
% Function: Version 67 is ULA/ULA MUSIC w/ integrated lidar
% Notes:    (1) MUSIC Algorithm separate; includes FB averaging
%           (2) Lidar uses Alden's linear regression algorithm w/
%               Capstoneociated functions
%           (3) Req. Functions:
%               point_servo_v2
%               read_range
%               tracker_step_v8
%               MusicAlg_v2
%           (4) Logan added sim mode
%           (5) Logan added tic toc
%           (6) New Globe Plotting
% Author:   Parker Reeves
% Date:     04/16/2026
% ======================================================================= %

clear all
clear r;

SIM_MODE = false;

% ====================== RANGEFINDER INITIALIZATION ===================== %

if SIM_MODE == false
    try
        u = udpport("IPV4", "LocalPort", 5005);
        r = raspi('169.254.67.1', 'analog', 'analog');
        if ~exist('s_az', 'var')
            s_az = servo(r, 13, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
        end
        if ~exist('s_el', 'var')
            s_el = servo(r, 12, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
        end
        opts = struct();

        point_servo_v2('reset');
        fprintf('Raspberry Pi detected\n');
    catch
        fprintf('Raspberry Pi not detected - LiDAR disabled\n');
        r = [];
        opts_in = struct();
    end
else
        fprintf('Sim Mode True - LiDAR disabled\n');
        r = [];
        opts_in = struct();
end


% ==================== ARRAY INITIALIZATION (UCA) ======================= %

c = 3e8;
f = 2.4e9;
lambda = c/f;

M = 4;
d = 0.5;
k = 2*pi/lambda;
delta = d*lambda;
J = fliplr(eye(M));

% ========================= SIGNAL RECEPTION / AUTO SIM MODE ============ %

if SIM_MODE == false

    try
        Y_1 = adi.FMComms5.Rx('uri','ip:192.168.1.101');
        Y_1.EnabledChannels = [1 2 3 4];

        Y_2 = adi.FMComms5.Rx('uri','ip:192.168.0.1');
        Y_2.EnabledChannels = [1 2 3 4];

        fprintf('Hardware detected - running in LIVE MODE\n');

    catch

        SIM_MODE = true;
        fprintf('Hardware not detected - running in SIMULATION MODE\n');

        % Simulated source trajectory parameters
        sim_az      = -60;
        sim_el      = -30;
        sim_az_step = 0.25;
        sim_el_step = 0.05;
        sim_snr     = 12;
        N_snap      = 32768;

    end

else

    sim_az      = -60;
    sim_el      = -30;
    sim_az_step = 0.25;
    sim_el_step = 0.05;
    sim_snr     = 12;
    N_snap      = 32768;
    fprintf('Sim Mode True - running in SIMULATION MODE\n');

end

phi_scan   = -90:1:90;
theta_scan = -90:1:90;

% =========================== INITIALIZE PLOTS =========================== %

figure;

subplot(2,1,1)
h1 = plot(phi_scan, zeros(size(phi_scan)), ...
          'LineWidth', 2, 'Color', [0 0.447 0.741]);
grid on;
title('ULA MUSIC Spectrum (Direct Element Space)');
xlabel('Elevation Angle (degrees)');
ylabel('Pseudo-spectrum (dB)');
xlim([-90 90]);

subplot(2,1,2)
h2 = plot(theta_scan, zeros(size(theta_scan)), ...
          'LineWidth', 2, 'Color', [0 0.447 0.741]);
grid on;
title('ULA MUSIC Spectrum (Direct Element Space)');
xlabel('Azimuth Angle (degrees)');
ylabel('Pseudo-spectrum (dB)');
xlim([-90 90]);

n_cycles = 0;
avg_time  = 0;

% ============================ 3D VISUALIZATION ========================= %
trail_len = 20;
trail_xyz = nan(trail_len, 3);
trail_idx = 0;
trail_count = 0;
rmax_cm = 300;

fig_globe = figure;

% Option 3: Spherical "radar globe" view
ax_globe = axes('Parent', fig_globe);
hold(ax_globe, 'on');
grid(ax_globe, 'on');
view(ax_globe, 3);
axis(ax_globe, 'equal');
xlim(ax_globe, [0 rmax_cm]);
ylim(ax_globe, [-rmax_cm rmax_cm]);
zlim(ax_globe, [-rmax_cm/2 rmax_cm]);
xlabel(ax_globe, 'X (cm)');
ylabel(ax_globe, 'Y (cm)');
zlabel(ax_globe, 'Z (cm)');
title(ax_globe, 'Option 3: Radar Globe View (Az/El Limited to +/-90 deg)');

% Render only the az/el field of view: az in [-90,90], el in [-90,90]
[az_grid, el_grid] = meshgrid(-90:3:90, -30:3:90);
xs = rmax_cm .* cosd(el_grid) .* cosd(az_grid);
ys = rmax_cm .* cosd(el_grid) .* sind(az_grid);
zs = rmax_cm .* sind(el_grid);
surf(ax_globe, xs, ys, zs, ...
     'FaceAlpha', 0.2, 'EdgeAlpha', 0.12, ...
     'FaceColor', [0.15 0.55 0.85], 'EdgeColor', [0.15 0.55 0.85]);

% Elevation "floor" across allowed azimuth
floor_el_deg = 0;
[az_floor, r_floor] = meshgrid(-90:3:90, linspace(0, rmax_cm, 40));
xf = r_floor .* cosd(floor_el_deg) .* cosd(az_floor);
yf = r_floor .* cosd(floor_el_deg) .* sind(az_floor);
zf = r_floor .* sind(floor_el_deg);
surf(ax_globe, xf, yf, zf, ...
     'FaceAlpha', 0.2, 'EdgeAlpha', 0.10, ...
     'FaceColor', [0.2 0.8 0.35], 'EdgeColor', [0.2 0.8 0.35]);

t_ref = linspace(-90, 90, 240);
% el = 0 arc across allowed azimuth
plot3(ax_globe, rmax_cm*cosd(t_ref), rmax_cm*sind(t_ref), zeros(size(t_ref)), ...
      '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
% az = 0 arc across allowed elevation
plot3(ax_globe, rmax_cm*cosd(t_ref), zeros(size(t_ref)), rmax_cm*sind(t_ref), ...
       '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);

h_globe_trail = plot3(ax_globe, nan, nan, nan, '-', 'LineWidth', 2, ...
                      'Color', [0.95 0.55 0.15]);
h_globe_pt = plot3(ax_globe, nan, nan, nan, 'o', 'MarkerSize', 8, ...
                   'MarkerFaceColor', [0.95 0.2 0.2], ...
                   'MarkerEdgeColor', [0.2 0.2 0.2]);
h_globe_ray = plot3(ax_globe, [0 nan], [0 nan], [0 nan], '-', ...
                    'LineWidth', 1.2, 'Color', [0.95 0.2 0.2]);

% Live readout in figure space (bottom-left), not on the globe axes
h_status_dist = annotation(fig_globe, 'textbox', [0.02 0.1 0.34 0.05], ...
                           'String', 'Estimated Distance: -- cm', ...
                           'FitBoxToText', 'off', ...
                           'HorizontalAlignment', 'left', ...
                           'VerticalAlignment', 'bottom', ...
                           'LineStyle', 'none', ...
                           'FontSize', 15, ...
                           'Color', [0.85 0.1 0.1]);  % default red

h_status_angles = annotation(fig_globe, 'textbox', [0.02 0.02 0.34 0.06], ...
                             'String', sprintf(['Estimated Azimuth:  -- deg\n', ...
                                                'Estimated Elevation: -- deg']), ...
                             'FitBoxToText', 'off', ...
                             'HorizontalAlignment', 'left', ...
                             'VerticalAlignment', 'bottom', ...
                             'LineStyle', 'none', ...
                             'FontSize', 15, ...
                             'Color', [0.1 0.1 0.1]);

% ============================== MAIN LOOP ============================== %

while true

    tic  % <<< START cycle timer

    spectrum_1 = zeros(size(theta_scan)); % ULAAZ Array
    spectrum_2 = zeros(size(phi_scan));   % ULAEL Array

    % ==================== DATA CAPTURE OR SIMULATION =================== %

    if SIM_MODE
        % Oscillate source back and forth across az and el
        sim_az = sim_az + sim_az_step;
        sim_el = sim_el + sim_el_step;
        if sim_az >  90 || sim_az < -90, sim_az_step = -sim_az_step; end
        if sim_el >  90 || sim_el < -90, sim_el_step = -sim_el_step; end

        a_az_true = exp(-1j*k*(0:M-1)'*delta*sind(sim_az));
        a_el_true = exp(-1j*k*(0:M-1)'*delta*sind(sim_el));

        noise_pwr = 10^(-sim_snr/10);
        s_sim_az  = (randn(1, N_snap) + 1j*randn(1, N_snap)) / sqrt(2);
        s_sim_el  = (randn(1, N_snap) + 1j*randn(1, N_snap)) / sqrt(2);

        X_1 = (a_az_true * s_sim_az + sqrt(noise_pwr) * (randn(M, N_snap) + 1j*randn(M, N_snap)) / sqrt(2))';  % (N_snap x M)
        X_2 = (a_el_true * s_sim_el + sqrt(noise_pwr) * (randn(M, N_snap) + 1j*randn(M, N_snap)) / sqrt(2))';  % (N_snap x M)

    else
        X_1 = fliplr(Y_1()); % ULAAZ Data
        X_2 = Y_2(); % ULAEL Data
    end

    En_1 = MusicAlg_v2(X_1,J); % ULAAZ Noise subspace
    En_2 = MusicAlg_v2(X_2,J); % ULAEL Noise subspace

% ===================== MUSIC SPECTRUM SEARCH (ULAEL) 2 ===================== %

    for i = 1:length(phi_scan)
        a_theta = exp(-1j*k*(0:M-1)'*delta*sind(phi_scan(i)));
        spectrum_2(i) = 1 / abs((a_theta' * (En_2 * En_2') * a_theta));
    end

    [~, max_idx] = max(10*log10(abs(spectrum_2)));
    est_EL = abs(phi_scan(max_idx));
    
% ===================== MUSIC SPECTRUM SEARCH (ULAAZ) 1 ===================== %

    for t = 1:length(theta_scan)
        a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));
        spectrum_1(t) = 1 / abs(a_scan'*(En_1*En_1')*a_scan);
    end

    [~, idx_peak] = max(10*log10(abs(spectrum_1)));
    est_AZ = theta_scan(idx_peak);

% =========================== VISUALIZATION ============================= %

    % fprintf('Estimated Azimuth: %.2f°\n', est_AZ);

    set(h1, 'YData', 10*log10(abs(spectrum_2)));
    set(h2, 'YData', 10*log10(abs(spectrum_1)));

    % ===================== LIDAR / SIM DISTANCE ======================== %

    if SIM_MODE
        dist_cm = norm([sim_az, sim_el]) * 10;  % fake distance for display
        fprintf('Estimated Distance: %.1f cm (simulated)\n', dist_cm);
    else
        az_in = est_AZ;      % your estimator output (pre +90 mapping)
        el_in = est_EL;    % your estimator output (el_offset applied internally)

        [cmd_az, cmd_el, status] = point_servo_v2(r, az_in, el_in, s_az, s_el);
        dist_m = read_range(u,4);
        dist_cm = dist_m*100;
        fprintf('az_in=%.2f el_in=%.2f cmd_az=%.2f cmd_el=%.2f dist=%.1f', az_in, el_in, cmd_az, cmd_el, dist_m);
        fprintf(['Estimated Distance(m): ', dist_m])
    end

    % ======== 3D VISUALIZATION UPDATE (SIM + LIVE) ======== %
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

    % Globe update
    set(h_globe_trail, 'XData', trail_xyz(trail_order,1), ...
                       'YData', trail_xyz(trail_order,2), ...
                       'ZData', trail_xyz(trail_order,3));
    set(h_globe_pt, 'XData', x_cm, 'YData', y_cm, 'ZData', z_cm);
    set(h_globe_ray, 'XData', [0 x_cm], 'YData', [0 y_cm], 'ZData', [0 z_cm]);
    if dist_cm < 500
        dist_color = [0.1 0.65 0.1];   % green
    else
        dist_color = [0.85 0.1 0.1];   % red
    end
    set(h_status_dist, 'String', sprintf('Estimated Distance: %.1f cm', dist_cm), ...
                       'Color', dist_color);
    set(h_status_angles, 'String', sprintf(['Estimated Azimuth:  %.2f deg\n', ...
                                            'Estimated Elevation: %.2f deg'], ...
                                            est_AZ, est_EL));
    drawnow limitrate

    fprintf('Estimated Azimuth: %.2f°\n', est_AZ);
    fprintf('Estimated Elevation: %.2f°\n', est_EL);

    if SIM_MODE
        fprintf('True Azimuth:      %.2f°\n', sim_az);
        fprintf('True Elevation:    %.2f°\n', sim_el);
    end

    % ====================== TICK-TOCK REPORT =========================== %

    elapsed   = toc;
    n_cycles  = n_cycles + 1;
    avg_time  = avg_time + (elapsed - avg_time) / n_cycles;  % running mean

    fprintf('DOA Cycle Time: %.4f s  (%.1f Hz)  |  Avg: %.4f s\n', ...
             elapsed, 1/elapsed, avg_time);

    pause(0.05)

end
