% Script    question_seven_Mean_v_SNR_MultiSource
% Purpose  	Calculate the Mean vs. SNR across 300 MC trials
%               for a Uniform Linear Array for Multiplr Sources
% Notes     (1) Number on Antennas: 4
%           (2) Number of Snapshots: 200
%           (3) Source Location: 60 & 75 deg
%           (4) SNR: 0 -> 30dB counting by 5dB
% Author    2026 DoA Team
% Date     	10/6/2025

clc; clear; close all;

M = 8;                   % Number of array elements
d = 0.5;                 % Element spacing (in wavelengths)
theta_true = [60 75];    % True signal directions (DOAs) in degrees
n_snapshots = 200;       % Number of temporal samples per trial
SNR_dB = 0:5:30;         % Range of SNR values to test (dB)
n_trials = 300;          % Number of Monte Carlo trials per SNR
c = 3e8; f = 2.4e9;      % Speed of light and operating frequency
lambda = c/f;            % Wavelength (m)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters
n_sources = length(theta_true);   % Number of signal sources

mean_est = zeros(size(SNR_dB));   % Estimation vector for mean DOA vs. SNR

for s = 1:length(SNR_dB)
    est_DOAs = zeros(n_sources, n_trials);  % Store estimated DOAs for each trial
    
    for mc = 1:n_trials
        A = zeros(M, n_sources);                                           % Initialize steering matrix
        for src = 1:n_sources
            A(:, src) = exp(-1j*k*(0:M-1)'*delta*sind(theta_true(src)));  % Steering vector for each source
        end
        
        s_sig = randn(n_sources, n_snapshots) + 1j*randn(n_sources, n_snapshots);  % Generate random complex source signals
        x = A*s_sig;                                                                % Signal received across array elements
        noise = (randn(M, n_snapshots) + 1j*randn(M, n_snapshots))/sqrt(2)*10^(-SNR_dB(s)/20);  % Complex Gaussian noise scaled by SNR
        X = x + noise;                                                              % Noisy received signal matrix
        
        Rxx = (X*X')/n_snapshots;               % Sample covariance matrix of received data
        [Evec, Eval] = eig(Rxx);                % Eigen-decomposition of covariance matrix
        [~, idx] = sort(diag(Eval), 'descend'); % Sort eigenvalues in descending order
        Evec = Evec(:, idx);                    % Sort eigenvectors accordingly
        En = Evec(:, n_sources+1:end);          % Extract noise subspace (for 2 sources)

        theta_scan = -90:0.2:90;                % Angle search grid (degrees)
        Pmusic = zeros(size(theta_scan));       % Initialize MUSIC spectrum
        for t = 1:length(theta_scan)
            a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));       % Steering vector for scan angle
            Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);                 % MUSIC pseudo-spectrum value
        end
        
        [~, locs] = findpeaks(real(Pmusic), theta_scan, 'SortStr','descend','MinPeakDistance',2);  % Find peaks in MUSIC spectrum
        locs = sort(locs(1:n_sources));                                  % Select top 2 peak locations
        est_DOAs(:, mc) = locs;                                          % Store estimated DOAs for this trial
    end
    
    mean_est(s) = mean(reshape(est_DOAs, 1, []));                        % Compute mean DOA across both sources for this SNR
    
%==================== Plotting ==============================
    figure;
    stem(1:n_trials, est_DOAs(1,:), 'b', 'filled', 'MarkerSize', 2); hold on;
    stem(1:n_trials, est_DOAs(2,:), 'g', 'filled', 'MarkerSize', 2);
    yline(theta_true(1), 'b--', 'LineWidth', 2);                        % True DOA reference line for source 1
    yline(theta_true(2), 'g--', 'LineWidth', 2);                        % True DOA reference line for source 2
    hold off;
    title(sprintf('MUSIC DOA Estimates at %d dB SNR', SNR_dB(s)));
    xlabel('Monte Carlo Trial');
    ylabel('Estimated DOA (°)');
    legend('Est. Source 1','Est. Source 2','True 60°','True 75°','Location','northwest');
    grid on;
    ylim([50 85]);                                                      % Focus on 50°–85° range
    
    fprintf('SNR = %2d dB --> Mean = %.2f°\n', ...
            SNR_dB(s), mean_est(s));
end

figure;
plot(SNR_dB, mean_est, 'o-', 'LineWidth', 2);                           % Plot mean DOA vs. SNR
hold on;
yline(theta_true(1), 'b--', 'LineWidth', 1.5);                          % True DOA reference line for 60°
yline(theta_true(2), 'g--', 'LineWidth', 1.5);                          % True DOA reference line for 75°
xlabel('SNR (dB)');
ylabel('Mean Estimated DOA (°)');
title('MUSIC DOA Mean Estimation vs SNR (Two Sources)');
legend('Mean Estimated DOA','True 60°','True 75°','Location','northwest');
grid on;




