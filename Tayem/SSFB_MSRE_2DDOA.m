% RMSE Performance Analysis: Signal-Space FB MUSIC
% Targets: Elevation (30 deg) and Azimuth (45 deg)
clear; clc; close all;

%% 1. Configuration
c = 3e8; f = 2.4e9; lambda = c/f;
d = lambda/2; R = lambda/2;
N_ula = 4; N_uca = 4; K = 500;

theta_true = 30; % Target Elevation
phi_true = 45;   % Target Azimuth

SNR_range = 0:5:30;   % SNR values to test (dB)
num_trials = 100;     % Monte Carlo iterations per SNR point

% Pre-allocate RMSE arrays
rmse_ele = zeros(length(SNR_range), 1);
rmse_azi = zeros(length(SNR_range), 1);

% Search grid parameters
search_res = 0.05; % High resolution search

%% 2. Monte Carlo Simulation
for s_idx = 1:length(SNR_range)
    SNR = SNR_range(s_idx);
    noise_std = 10^(-SNR/20) / sqrt(2);
    
    sq_err_ele = zeros(num_trials, 1);
    sq_err_azi = zeros(num_trials, 1);
    
    fprintf('Running SNR = %d dB...\n', SNR);
    
    for trial = 1:num_trials
        % --- Signal Generation ---
        sig = exp(1i*2*pi*f*(0:K-1)/f); 
        
        %% Step A: Elevation (ULA) with Signal-Space FB
        A_ula = exp(1i*2*pi*d/lambda * (0:N_ula-1)' * cosd(theta_true));
        X_ula = A_ula * sig + noise_std * (randn(N_ula,K) + 1i*randn(N_ula,K));
        
        J_ula = fliplr(eye(N_ula));
        X_fb_ula = [X_ula, J_ula * conj(X_ula)]; % Augmented Data
        [U_ula, ~, ~] = svd(X_fb_ula, 'econ');
        En_ula = U_ula(:, 2:end);
        
        % Local Search around true theta
        th_vec = theta_true-2 : search_res : theta_true+2;
        p_ele = zeros(size(th_vec));
        for i = 1:length(th_vec)
            a = exp(1i*2*pi*d/lambda * (0:N_ula-1)' * cosd(th_vec(i)));
            p_ele(i) = 1 / (a' * (En_ula * En_ula') * a);
        end
        [~, m_idx] = max(abs(p_ele));
        theta_est = th_vec(m_idx);
        sq_err_ele(trial) = (theta_est - theta_true)^2;
        
        %% Step B: Azimuth (UCA) with Signal-Space FB
        angle_n = 2*pi*(0:N_uca-1)/N_uca; 
        A_uca = exp(1i*2*pi*R/lambda * sind(theta_true) * cos(deg2rad(phi_true) - angle_n'));
        X_uca = A_uca * sig + noise_std * (randn(N_uca,K) + 1i*randn(N_uca,K));
        
        J_uca = fliplr(eye(N_uca));
        X_fb_uca = [X_uca, J_uca * conj(X_uca)];
        [U_uca, ~, ~] = svd(X_fb_uca, 'econ');
        En_uca = U_uca(:, 2:end);
        
        % Local Search around true phi (using estimated theta)
        ph_vec = phi_true-2 : search_res : phi_true+2;
        p_azi = zeros(size(ph_vec));
        for i = 1:length(ph_vec)
            a = exp(1i*2*pi*R/lambda * sind(theta_est) * cos(deg2rad(ph_vec(i)) - angle_n'));
            p_azi(i) = 1 / (a' * (En_uca * En_uca') * a);
        end
        [~, m_idx] = max(abs(p_azi));
        phi_est = ph_vec(m_idx);
        sq_err_azi(trial) = (phi_est - phi_true)^2;
    end
    
    % Calculate RMSE for this SNR
    rmse_ele(s_idx) = sqrt(mean(sq_err_ele));
    rmse_azi(s_idx) = sqrt(mean(sq_err_azi));
end

%% 3. Plotting the Results
figure('Color', 'w', 'Name', 'RMSE Performance');
semilogy(SNR_range, rmse_ele, 'o-b', 'LineWidth', 2, 'MarkerSize', 8); hold on;
semilogy(SNR_range, rmse_azi, 's-r', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
xlabel('SNR (dB)', 'FontSize', 12);
ylabel('RMSE (Degrees)', 'FontSize', 12);
title(['RMSE vs SNR (', num2str(num_trials), ' Trials)'], 'FontSize', 14);
legend('Elevation RMSE (30^{\circ})', 'Azimuth RMSE (45^{\circ})', 'Location', 'northeast');
set(gca, 'FontSize', 11);
