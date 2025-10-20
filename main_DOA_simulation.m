%% Simulation Parameters
% --- Array Parameters ---
M = 4;                      % Number of antenna elements
lambda = 0.125;             % Wavelength of the signal
R = lambda / (2 * sin(pi/M)); % Array radius, ensures half-wavelength spacing
k = 2 * pi / lambda;        % Wavenumber

% --- Signal Parameters ---
D = 1;                      % Number of sources
theta_source = 75;          % Azimuth of source (degrees)
phi_source = 90;            % Elevation of source (degrees)
N = 100;                    % Number of snapshots (samples)

% --- Simulation Control ---
SNR_dB = -10:1:10;          % SNR range in dB
num_trials = 100;           % Number of Monte Carlo trials for averaging

% --- Antenna Pattern Parameters ---
q_directional_1 = 0.25;        % Directional pattern order (cos^q)

%% Pre-computation & Initialization
% Calculate antenna element positions on the xy-plane
ant_angles = (0:M-1) * (2*pi/M);
ant_pos = R * [cos(ant_angles); sin(ant_angles); zeros(1, M)];

% Angular search space for MUSIC algorithm
theta_scan = 0:2.5:180;
phi_scan = 90;

rmse_directional = zeros(size(SNR_dB));

%% Main Simulation Loop for RMSE Plot
fprintf('Starting DOA Estimation Simulation for RMSE Plot...\n');
for i_snr = 1:length(SNR_dB)
    snr_val = SNR_dB(i_snr);
    fprintf('  Running SNR = %.1f dB\n', snr_val);
    
    sq_err_directional = zeros(1, num_trials);

    for i_trial = 1:num_trials
        % Generate source signal and noise
        s = sqrt(1/2) * (randn(D, N) + 1j * randn(D, N));
        snr_linear = 10^(snr_val / 10);
        noise_power = 1 / snr_linear;
        n = sqrt(noise_power/2) * (randn(M, N) + 1j * randn(M, N));
        
        a_directional = create_uca_steering_vector(theta_source, phi_source, k, ant_pos, 'directional', q_directional_1);
        x_directional = a_directional * s + n;

        theta_est_directional = music_algorithm(x_directional, D, k, ant_pos, theta_scan, phi_scan, 'directional', q_directional_1);

        sq_err_directional(i_trial) = (theta_est_directional - theta_source)^2;
    end

    rmse_directional(i_snr) = sqrt(mean(sq_err_directional));
end
fprintf('Simulation finished.\n');

%% Plotting Results - RMSE vs SNR
semilogy(SNR_dB, rmse_directional, '-^', 'LineWidth', 2, 'DisplayName', ['Directional (cos^' num2str(q_directional_1) ')']);
grid on;
xlabel('Signal-to-Noise Ratio (SNR) [dB]');
ylabel('Root Mean Square Error (RMSE) [degrees]');
title(['DOA Estimation Performance for a 4-Element UCA (N=', num2str(N), ')']);
legend('show', 'Location', 'southwest');
ylim([1e-2 1e2]);