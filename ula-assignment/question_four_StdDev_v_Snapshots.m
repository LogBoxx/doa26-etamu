% Script    question_four_StdDev_v_Snapshots
% Purpose  	Calculate the Standard Deviation vs. Number of Snapshots across 300 MC trials
%               for a Uniform Linear Array
% Notes     (1) Number on Antennas: 10
%           (2) SNR = 5dB
%           (3) Number of Snapshots: 100 -> 1000 counting by 100
%           (4) Source Location: 55 deg
% Author    2026 DoA Team
% Date     	10/6/2025

clc; clear; close all;

M = 10;                  % Number of array elements
d = 0.5;                 % Element spacing (in wavelengths)
theta_true = 55;         % True signal direction (DOA) in degrees
SNR_dB = 5;              % Fixed SNR value (dB)
n_trials = 300;          % Number of Monte Carlo trials per snapshot count
c = 3e8; f = 2.4e9;      % Speed of light and operating frequency
lambda = c/f;            % Wavelength (m)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters
snapshot_values = 100:100:1000;    % Number of snapshots to test (100, 200, ..., 1000)

std_est = zeros(size(snapshot_values));    % Estimation vector for standard deviation vs. snapshots

for n = 1:length(snapshot_values)
    n_snapshots = snapshot_values(n);       % Current number of snapshots
    est_DOAs = zeros(1, n_trials);          % Store estimated DOAs for each trial
    
    for mc = 1:n_trials
        a = exp(-1j*k*(0:M-1)'*delta*sind(theta_true));                          % Steering vector for true DOA
        s_sig = randn(1, n_snapshots) + 1j*randn(1, n_snapshots);                % Generate random complex source signal
        x = a*s_sig;                                                             % Signal received across array elements
        noise = (randn(M, n_snapshots) + 1j*randn(M, n_snapshots))/sqrt(2)*10^(-SNR_dB/20); % Complex Gaussian noise
        X = x + noise;                                                           % Noisy received signal matrix
        
        Rxx = (X*X')/n_snapshots;               % Sample covariance matrix of received data
        [Evec, Eval] = eig(Rxx);                % Eigen-decomposition of covariance matrix
        [~, idx] = sort(diag(Eval), 'descend'); % Sort eigenvalues in descending order
        Evec = Evec(:, idx);                    % Sort eigenvectors accordingly
        En = Evec(:, 2:end);                    % Extract noise subspace (for 1 source)

        theta_scan = -90:0.2:90;                % Angle search grid (degrees)
        Pmusic = zeros(size(theta_scan));       % Initialize MUSIC spectrum
        for t = 1:length(theta_scan)
            a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));              % Steering vector for scan angle
            Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);                        % MUSIC pseudo-spectrum value
        end

        [~, idx_peak] = max(Pmusic);            % Find peak index in MUSIC spectrum
        est_DOAs(mc) = theta_scan(idx_peak);    % Store estimated DOA for this trial
    end
    
    std_est(n) = std(est_DOAs);                 % Compute standard deviation for current snapshot count

    fprintf('Snapshots = %4d --> Mean = %.2f°, Std = %.2f°\n', ...
            n_snapshots, mean(est_DOAs), std(est_DOAs));
end

%==================== Plotting ==============================
figure;
plot(snapshot_values, std_est, 'o-', 'LineWidth', 2);
xlabel('Number of Snapshots');
ylabel('Standard Deviation of Estimated DOA (°)');
title('MUSIC DOA Estimation Performance vs Number of Snapshots');
grid on;


