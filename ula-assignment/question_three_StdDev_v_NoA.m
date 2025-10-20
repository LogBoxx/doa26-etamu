% Script    question_three_StdDev_v_NoA
% Purpose  	Calculate the Standard Deviation vs. Number of Antennas across 300 MC trials
%               for a Uniform Linear Array
% Notes     (1) Number on Antennas: 4 -> 24 counting by 2
%           (2) Number of Snapshots: 200
%           (3) Source Location: 50 deg
%           (4) SNR: 10dB
% Author    2026 DoA Team
% Date     	10/6/2025

clc; clear; close all;

theta_true = 50;         % True signal direction (DOA) in degrees
n_snapshots = 200;       % Number of temporal samples per trial
SNR_dB = 10;             % Fixed SNR value (dB)
n_trials = 300;          % Number of Monte Carlo trials per antenna count
c = 3e8; f = 2.4e9;      % Speed of light and operating frequency
lambda = c/f;            % Wavelength (m)
d = 0.5;                 % Element spacing (in wavelengths)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters
M_values = 2:2:24;       % Number of antennas to test (2, 4, 6, ..., 24)

std_est = zeros(size(M_values));     % Estimation vector for standard deviation vs. antenna count

for m = 1:length(M_values)
    M = M_values(m);                 % Current number of antennas
    est_DOAs = zeros(1, n_trials);   % Store estimated DOAs for each trial
    
    for mc = 1:n_trials
        a = exp(-1j*k*(0:M-1)'*delta*sind(theta_true));                                     % Steering vector for true DOA
        s_sig = randn(1, n_snapshots) + 1j*randn(1, n_snapshots);                           % Generate random complex source signal
        x = a*s_sig;                                                                        % Signal received across array elements
        noise = (randn(M, n_snapshots) + 1j*randn(M, n_snapshots))/sqrt(2)*10^(-SNR_dB/20); % Complex Gaussian noise
        X = x + noise;                                                                      % Noisy received signal matrix
        
        Rxx = (X*X')/n_snapshots;               % Sample covariance matrix of received data
        [Evec, Eval] = eig(Rxx);                % Eigen-decomposition of covariance matrix
        [~, idx] = sort(diag(Eval), 'descend'); % Sort eigenvalues in descending order
        Evec = Evec(:, idx);                    % Sort eigenvectors accordingly
        En = Evec(:, 2:end);                    % Extract noise subspace (for 1 source)
        
        theta_scan = -90:0.2:90;                % Angle search grid (degrees)
        Pmusic = zeros(size(theta_scan));       % Initialize MUSIC spectrum
        for t = 1:length(theta_scan)
            a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));      % Steering vector for scan angle
            Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);                % MUSIC pseudo-spectrum value
        end
        
        [~, idx_peak] = max(Pmusic);            % Find peak index in MUSIC spectrum
        est_DOAs(mc) = theta_scan(idx_peak);    % Store estimated DOA for this trial
    end
    
    std_est(m) = std(est_DOAs);                 % Compute standard deviation for current antenna count
    
    fprintf('M = %2d --> Mean = %.2f°, Std = %.2f°\n', ...
            M, mean(est_DOAs), std(est_DOAs));
end

%==================== Plotting ==============================
figure;
plot(M_values, std_est, 'o-', 'LineWidth', 2);
xlabel('Number of Antennas');
ylabel('Standard Deviation of Estimated DOA (°)');
title('MUSIC DOA Estimation Performance vs Number of Antennas');
grid on;

