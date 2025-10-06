% Script    question_one_StdDev_v_SNR
% Purpose  	Calculate the Standard Deviation vs. SNR across 300 MC trials
%               for a Uniform Linear Array
% Notes     (1) Number on Antennas: 4
%           (2) Number of Snapshots: 200
%           (3) Source Location: 30 deg
%           (4) SNR: 0 -> 30dB counting by 5dB
% Author    2026 DoA Team
% Date     	10/6/2025

clc; clear; close all;

M = 4;                   % Number of array elements
d = 0.5;                 % Element spacing (in wavelengths)
theta_true = 30;         % True signal direction in degrees
n_snapshots = 200;       % Number of samples per trial
SNR_dB = 0:5:30;         % Range of SNR values to test (dB)
n_trials = 300;          % Number of Monte Carlo trials per SNR
c = 3e8; f = 2.4e9;      % Speed of light and operating frequency
lambda = c/f;            % Wavelength (m)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters

std_est = zeros(size(SNR_dB));      % Estimation vector for standard deviation vs. SNR

for s = 1:length(SNR_dB)
    est_DOAs = zeros(1, n_trials);  % Store estimated DOAs for each trial
    
    for mc = 1:n_trials
        a = exp(-1j*k*(0:M-1)'*delta*sind(theta_true));                                         % Steering vector for true DOA
        s_sig = randn(1, n_snapshots) + 1j*randn(1, n_snapshots);                               % Generate random complex source signal
        x = a*s_sig;                                                                            % Signal received across array elements
        noise = (randn(M, n_snapshots) + 1j*randn(M, n_snapshots))/sqrt(2)*10^(-SNR_dB(s)/20);  % Complex Gaussian noise scaled by SNR
        X = x + noise;                                                                          % Noisy received signal matrix
        
        Rxx = (X*X')/n_snapshots;               % Sample covariance matrix of received data
        [Evec, Eval] = eig(Rxx);                % Eigen-decomposition of covariance matrix
        [~, idx] = sort(diag(Eval), 'descend'); % Sort eigenvalues in descending order
        Evec = Evec(:, idx);                    % Sort eigenvectors accordingly
        En = Evec(:, 2:end);                    % Extract noise subspace (for 1 source)
        
        % --- MUSIC Spectrum Computation ---
        theta_scan = -90:0.2:90;                % Angle search grid (degrees)
        Pmusic = zeros(size(theta_scan));       % Initialize MUSIC spectrum
        for t = 1:length(theta_scan)
            a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));      % Steering vector for scan angle
            Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);                % MUSIC pseudo-spectrum value
        end
        
        % --- DOA Estimation ---
        [~, idx_peak] = max(Pmusic);            % Find peak index in MUSIC spectrum
        est_DOAs(mc) = theta_scan(idx_peak);    % Store estimated DOA for this trial
    end
    
    std_est(s) = std(est_DOAs);                 % Compute spread of DOA estimates
    
%==================== Plotting ==============================
    figure;
    stem(1:n_trials, est_DOAs, 'filled', 'MarkerSize', 2);  % Plot DOA estimates per trial
    hold on;
    yline(theta_true, 'r', 'LineWidth', 2);                 % True DOA reference line
    hold off;
    title(sprintf('MUSIC DOA Estimates at %d dB SNR', SNR_dB(s)));
    xlabel('Monte Carlo Trial');
    ylabel('Estimated DOA (°)');
    grid on;
    ylim([25    35]); 
    
    fprintf('SNR = %2d dB --> Mean = %.2f°, Std = %.2f°\n', ...
            SNR_dB(s), mean(est_DOAs), std(est_DOAs));
end

figure;
plot(SNR_dB, std_est, 'o-', 'LineWidth', 2);     % Plot performance curve
xlabel('SNR (dB)');
ylabel('Standard Deviation of Estimated DOA (°)');
title('MUSIC DOA Estimation Performance vs SNR');
grid on;                                         

