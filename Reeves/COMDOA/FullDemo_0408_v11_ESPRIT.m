% ======================================================================= %
% Script:   FullDemo_0408_v12_ESPRIT
% Function: Version 12 - ULA/ULA ESPRIT w/ integrated lidar
% Notes:    (1) ESPRIT replaces MUSIC for DOA estimation
%           (2) Spatial spectrum computed separately for display only
%           (3) FB averaging applied to covariance before ESPRIT
%           (4) Lidar uses Alden's linear regression algorithm w/
%               Capstoneociated functions
%           (5) Req. Functions:
%               get_range_v7
%               lidar_read_tf02_v7
%               tracker_step_v7
%           (6) Sim mode, tic toc retained from v11
% Author:   Parker Reeves / Logan
% Date:     04/08/2026
% ======================================================================= %

clear all
clear r;

SIM_MODE = false;
num_trials = 500;
m_AZ = zeros(1,num_trials);

% ====================== RANGEFINDER INITIALIZATION ===================== %

if SIM_MODE == false
    try
        r = raspi('169.254.52.8','analog','analog');
        s_az = servo(r,13,'MinPulseDuration',5.44e-4,'MaxPulseDuration',2.40e-3);
        s_el = servo(r,12,'MinPulseDuration',5.44e-4,'MaxPulseDuration',2.40e-3);

        opts = struct();
        opts.el_offset = 7;      % tune if needed
        opts.max_move_az = 10;   % tune if needed
        opts.max_move_el = 5;    % tune if needed

        get_range_v6('reset');
        
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

% ==================== ARRAY INITIALIZATION ============================= %

c      = 3e8;
f      = 2.4e9;
lambda = c/f;

M     = 4;
d     = 0.5;        % half-wavelength spacing
k     = 2*pi/lambda;
delta = d*lambda;
J     = fliplr(eye(M));

num_sources = 1;    % number of sources to estimate

% ========================= SIGNAL RECEPTION / AUTO SIM MODE ============ %

if SIM_MODE == false

    try
        Y_1 = adi.FMComms5.Rx('uri','ip:192.168.1.101');
        Y_1.EnabledChannels = [1 2 3 4];
        Y_1.SamplesPerFrame = 2^5;

        % Y_2 = adi.FMComms5.Rx('uri','ip:192.168.0.101');
        % Y_2.EnabledChannels = [1 2 3 4];
        % Y_2.SamplesPerFrame = 2^5;
        % 
        % fprintf('Hardware detected - running in LIVE MODE\n');

    catch

        SIM_MODE = true;
        fprintf('Hardware not detected - running in SIMULATION MODE\n');

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

% Averaging buffer for ESPRIT estimates (same Navg idea as Logan's demo)
Navg   = 10;
az_hist = nan(Navg, num_sources);
el_hist = nan(Navg, num_sources);

% =========================== INITIALIZE PLOTS =========================== %

figure;

subplot(2,1,1)
h1 = plot(phi_scan, zeros(size(phi_scan)), ...
          'LineWidth', 1, 'Color', [0 0.447 0.741]);
hold on;
h1_mark = xline(0, '--r', 'LineWidth', 2.0);
hold off;
grid on;
title('ULA ESPRIT Spatial Spectrum - Elevation');
xlabel('Elevation Angle (degrees)');
ylabel('Spectrum (dB)');
xlim([-90 90]);

subplot(2,1,2)
h2 = plot(theta_scan, zeros(size(theta_scan)), ...
          'LineWidth', 1, 'Color', [0 0.447 0.741]);
hold on;
h2_mark = xline(0, '--r', 'LineWidth', 2.0);
hold off;
grid on;
title('ULA ESPRIT Spatial Spectrum - Azimuth');
xlabel('Azimuth Angle (degrees)');
ylabel('Spectrum (dB)');
xlim([-90 90]);

n_cycles = 0;
avg_time  = 0;

% ============================== MAIN LOOP ============================== %

for o = 1:num_trials

    tic

    spectrum_1 = zeros(size(theta_scan)); % AZ display spectrum
    spectrum_2 = zeros(size(phi_scan));   % EL display spectrum

    % ==================== DATA CAPTURE OR SIMULATION =================== %

    if SIM_MODE
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
        X_1 = Y_1(); % ULAAZ Data  (N_snap x M)
        %X_1 = fliplr(X_1);
        % X_2 = Y_2(); % ULAEL Data  (N_snap x M)
    end

    % ===================== ESPRIT DOA ESTIMATION ======================= %
    % Data is (N_snap x M), ESPRIT expects (M x N_snap), so transpose.

    [est_AZ, spectrum_1] = esprit_doa(X_1.', num_sources, d, theta_scan);
    % [est_EL, spectrum_2] = esprit_doa(X_2.', num_sources, d, phi_scan);

    % Running average (same as Logan's Navg buffer)
    az_hist = [az_hist(2:end,:); est_AZ(:).'];
    % el_hist = [el_hist(2:end,:); est_EL(:).'];
    est_AZ_avg = mean(az_hist, 1, 'omitnan');
    % est_EL_avg = mean(el_hist, 1, 'omitnan');

    % =========================== VISUALIZATION ========================= %

    % set(h1, 'YData', spectrum_1);
    % h1_mark.Value = est_EL_avg(1);

    set(h2, 'YData', spectrum_2);
    h2_mark.Value = est_AZ_avg(1);

    drawnow limitrate

    % ===================== LIDAR / SIM DISTANCE ======================== %

    if SIM_MODE
        dist_cm = norm([sim_az, sim_el]) * 10;
        fprintf('Estimated Distance: %.1f cm (simulated)\n', dist_cm);
    else
        % az_in = est_AZ_avg(1);
        % el_in = est_EL_avg(1);

        % [dist_cm, strength, cmd_az, cmd_el, status] = ...
        %     get_range_v6(r, az_in, 0, s_az, s_el, opts); %%%
        % 
        % fprintf('az_in=%.2f el_in=%.2f cmd_az=%.2f cmd_el=%.2f dist=%.1f str=%.0f no_action=%d\n', ...
        %     az_in, el_in, cmd_az, cmd_el, dist_cm, strength, status.track.no_action);
        %fprintf('Estimated Distance: %.1f cm\n', dist_cm);
    end

    fprintf('Estimated Azimuth:   %.2f deg  (avg: %.2f)\n', est_AZ(1), est_AZ_avg(1));
    m_AZ(o) = est_AZ_avg(1);
    %fprintf('Estimated Elevation: %.2f deg  (avg: %.2f)\n', est_EL(1), est_EL_avg(1));

    if SIM_MODE
        fprintf('True Azimuth:      %.2f deg\n', sim_az);
        fprintf('True Elevation:    %.2f deg\n', sim_el);
    end

    % ====================== TICK-TOCK REPORT =========================== %

    elapsed   = toc;
    n_cycles  = n_cycles + 1;
    avg_time  = avg_time + (elapsed - avg_time) / n_cycles;

    fprintf('DOA Cycle Time: %.4f s  (%.1f Hz)  |  Avg: %.4f s\n', ...
             elapsed, 1/elapsed, avg_time);

    pause(0.05)

end


% ======================================================================= %
% Local Function: esprit_doa
%
% Inputs:
%   rx_m        - (M x N) complex data matrix, M antennas, N snapshots
%   num_sources - number of sources to estimate
%   d           - element spacing in wavelengths (0.5 = half-wavelength)
%   scan_angles - vector of angles (deg) for display spectrum only
%
% Outputs:
%   doa_deg     - estimated DOA(s) in degrees (1 x num_sources)
%   sp_dB       - spatial spectrum in dB over scan_angles (for display)
% ======================================================================= %
function [doa_deg, sp_dB] = esprit_doa(rx_m, num_sources, d, scan_angles)

    [M, N] = size(rx_m);

    % Covariance with FB averaging
    R = (rx_m * rx_m') / N;
    J = flipud(eye(M));
    R = 0.5 * (R + J * conj(R) * J);

    % Eigendecomposition, sort descending
    [U, D] = eig(R);
    [~, idx] = sort(diag(D), 'descend');
    U = U(:, idx);
    Es = U(:, 1:num_sources);   % signal subspace

    % ESPRIT selection matrices (overlapping subarrays)
    J1 = [eye(M-1), zeros(M-1, 1)];
    J2 = [zeros(M-1, 1), eye(M-1)];

    E1 = J1 * Es;
    E2 = J2 * Es;

    % Solve for rotation matrix Psi
    Psi = pinv(E1) * E2;
    ev  = eig(Psi);

    % Convert eigenvalue phases to angles
    u = angle(ev) / (2 * pi * d);
    u = max(min(real(u), 1), -1);   % clamp to valid sind range
    doa_deg = sort(asind(u)).';      % (1 x num_sources)

    % Spatial spectrum for display (Capon/CBF using R, not MUSIC pseudospectrum)
    pos = (-(M-1)/2 : (M-1)/2).';
    sp  = zeros(size(scan_angles));
    for i = 1:numel(scan_angles)
        th   = scan_angles(i) * pi / 180;
        a    = exp(1j * 2 * pi * d * pos * sin(th));
        sp(i) = real(a' * R * a);
    end
    sp    = sp ./ max(sp + eps);
    sp_dB = 10 * log10(sp);

end

figure
plot(m_AZ,'rx','MarkerSize',8,'LineWidth',1.5)
xlim([0,num_trials])
title('ULA AZ ESPRIT')
xlabel('Trial Number')
ylabel('Azimuth Estimation')