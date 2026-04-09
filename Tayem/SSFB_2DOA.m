% MUSIC Algorithm: Split Array (ULA for Elevation, UCA for Azimuth)
% Method: Signal-Space Forward-Backward (FB) Averaging
clear; clc; close all;

%% 1. System Parameters
c = 3e8;                % Speed of light
f = 2.4e9;              % Operating Frequency
lambda = c/f;           % Wavelength
d = lambda/2;           % ULA Element spacing
R = lambda/2;           % UCA Radius
N_ula = 4;              % ULA elements (Z-axis)
N_uca = 4;              % UCA elements (XY-plane)
SNR = 20;               % SNR in dB
K = 500;                % Number of snapshots

% True Source Direction
theta_true = 30;        % Elevation (deg)
phi_true = 45;          % Azimuth (deg)

% Signal & Noise Generation
s = exp(1i*2*pi*f*(0:K-1)/f); % Reference signal
noise_std = 10^(-SNR/20) / sqrt(2);

%% 2. Elevation Estimation (ULA)
theta_range = 0:0.1:180;
n_ula = (0:N_ula-1)';
A_ula_true = exp(1i*2*pi*d/lambda * n_ula * cosd(theta_true));
X_ula = A_ula_true * s + noise_std * (randn(N_ula,K) + 1i*randn(N_ula,K));

% --- SIGNAL-SPACE FB AUGMENTATION (ULA) ---
J_ula = fliplr(eye(N_ula));
Z_ula = [X_ula, J_ula * conj(X_ula)]; % Augmented matrix

% SVD for Noise Subspace
[U_ula, ~, ~] = svd(Z_ula, 'econ');
En_ula = U_ula(:, 2:end); 

% MUSIC Spectrum for Elevation
P_ele = zeros(size(theta_range));
for i = 1:length(theta_range)
    a = exp(1i*2*pi*d/lambda * n_ula * cosd(theta_range(i)));
    P_ele(i) = 1 / (a' * (En_ula * En_ula') * a);
end

[~, idx_ele] = max(abs(P_ele));
theta_est = theta_range(idx_ele);

%% 3. Azimuth Estimation (UCA)
phi_range = 0:0.1:360;
angle_n = 2*pi*(0:N_uca-1)/N_uca; 
A_uca_true = exp(1i*2*pi*R/lambda * sind(theta_true) * cos(deg2rad(phi_true) - angle_n'));
X_uca = A_uca_true * s + noise_std * (randn(N_uca,K) + 1i*randn(N_uca,K));

% --- SIGNAL-SPACE FB AUGMENTATION (UCA) ---
J_uca = fliplr(eye(N_uca));
Z_uca = [X_uca, J_uca * conj(X_uca)];

% SVD for Noise Subspace
[U_uca, ~, ~] = svd(Z_uca, 'econ');
En_uca = U_uca(:, 2:end);

% MUSIC Spectrum for Azimuth (using theta_est from Step 1)
P_azi = zeros(size(phi_range));
for i = 1:length(phi_range)
    a = exp(1i*2*pi*R/lambda * sind(theta_est) * cos(deg2rad(phi_range(i)) - angle_n'));
    P_azi(i) = 1 / (a' * (En_uca * En_uca') * a);
end

[~, idx_azi] = max(abs(P_azi));
phi_est = phi_range(idx_azi);

%% 4. Visualization & Output
fprintf('--- DOA Estimation Results ---\n');
fprintf('True Elevation: %.2f°, Estimated: %.2f°\n', theta_true, theta_est);
fprintf('True Azimuth:   %.2f°, Estimated: %.2f°\n', phi_true, phi_est);

figure('Color', 'w');
subplot(2,1,1);
plot(theta_range, 10*log10(abs(P_ele)/max(abs(P_ele))), 'b', 'LineWidth', 1.5);
hold on; xline(theta_true, 'r--', 'True'); hold off;
title(['ULA Elevation Spectrum (Est: ', num2str(theta_est), '^{\circ})']);
grid on; ylabel('dB'); xlabel('\theta (deg)');

subplot(2,1,2);
plot(phi_range, 10*log10(abs(P_azi)/max(abs(P_azi))), 'Color', [0 0.5 0], 'LineWidth', 1.5);
hold on; xline(phi_true, 'r--', 'True'); hold off;
title(['UCA Azimuth Spectrum (Est: ', num2str(phi_est), '^{\circ})']);
grid on; ylabel('dB'); xlabel('\phi (deg)');
