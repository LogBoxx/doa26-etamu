% ======================================================================= %
% Script:   FullDemo_0408_v11
% Function: Version 11 is ULA/ULA MUSIC w/ integrated lidar
% Notes:    (1) MUSIC Algorithm separate; includes FB averaging
%           (2) Lidar uses Alden's linear regression algorithm w/
%               Capstoneociated functions
%           (3) Req. Functions:
%               get_range_v5_5
%               lidar_read_tf02_v5_5
%               tracker_step_v5_5
%               MusicAlg_v2
%           (4) Logan added sim mode
%           (5) Logan added tic toc
%           (6) New steering vector
% Author:   Parker Reeves
% Date:     04/08/2026
% ======================================================================= %

clear all
clear r;

SIM_MODE = true;

% ====================== RANGEFINDER INITIALIZATION ===================== %

if SIM_MODE == false
    try
        r = raspi('169.254.52.8','analog','analog');
        s_az = servo(r,13,'MinPulseDuration',5.44e-4,'MaxPulseDuration',2.40e-3);
        s_el = servo(r,12,'MinPulseDuration',5.44e-4,'MaxPulseDuration',2.40e-3);

        opts = struct();
        opts.el_offset = 0;      % tune if needed
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
        Y_1 = adi.FMComms5.Rx('uri','ip:192.168.0.101');
        Y_1.EnabledChannels = [1 2 3 4];

        Y_2 = adi.FMComms5.Rx('uri','ip:192.168.1.101');
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
        X_1 = Y_1(); % ULAAZ Data
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
    est_EL = phi_scan(max_idx);
    
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

    drawnow limitrate

    % ===================== LIDAR / SIM DISTANCE ======================== %

    if SIM_MODE
        dist_cm = norm([sim_az, sim_el]) * 10;  % fake distance for display
        fprintf('Estimated Distance: %.1f cm (simulated)\n', dist_cm);
    else
        az_in = est_AZ;      % your estimator output (pre +90 mapping)
        el_in = est_EL;    % your estimator output (el_offset applied internally)

        [dist_cm, strength, cmd_az, cmd_el, status] = ...
            get_range_v7(r, az_in, el_in, s_az, s_el, opts);

        fprintf('az_in=%.2f el_in=%.2f cmd_az=%.2f cmd_el=%.2f dist=%.1f str=%d no_action=%d\n', ...
            az_in, el_in, cmd_az, cmd_el, dist_cm, strength, status.track.no_action);
        fprintf(['Estimated Distance: ', dist_cm])
    end

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
