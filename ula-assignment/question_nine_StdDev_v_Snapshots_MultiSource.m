% Script    question_nine_StdDev_v_Snapshots_MultiSource
% Purpose  	Calculate the Standard Deviation vs. Number of Snapshots across 300 MC trials
%               for a Uniform Linear Array for Multiple Sources
% Notes     (1) Number of Antennas: 10
%           (2) SNR = 5dB
%           (3) Number of Snapshots: 100 -> 1000 counting by 100
%           (4) Source Locations: 60 & 75 deg
% Author    2026 DoA Team
% Date     	10/6/2025

clc; clear; close all;

M = 10;                  % Number of array elements
d = 0.5;                 % Element spacing (in wavelengths)
theta_true = [60 75];    % True signal directions (DOAs) in degrees
SNR_dB = 5;              % Fixed SNR value (dB)
n_trials = 300;          % Number of Monte Carlo trials per snapshot count
c = 3e8; f = 2.4e9;      % Speed of light and operating frequency
lambda = c/f;            % Wavelength (m)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters
snapshot_values = 100:100:1000;    % Number of snapshots to test
n_sources = length(theta_true);    % Number of signal sources

std_est = zeros(size(snapshot_values));    % Estimation vector for standard deviation vs. snapshots

for n = 1:length(snapshot_values)
    n_snapshots = snapshot_values(n);       % Current number of snapshots
    est_DOAs = zeros(n_sources, n_trials);  % Store estimated DOAs for each trial
    
    for mc = 1:n_trials
        A = zeros(M, n_sources);                                            % Initialize steering matrix
        for src = 1:n_sources
            A(:, src) = exp(-1j*k*(0:M-1)'*delta*sind(theta_true(src)));   % Steering vector for each source
        end
        
        s_sig = randn(n_sources, n_snapshots) + 1j*randn(n_sources, n_snapshots);   % Generate random complex source signals
        x = A*s_sig;                                                                 % Signal received across array elements
        noise = (randn(M, n_snapshots) + 1j*randn(M, n_snapshots))/sqrt(2)*10^(-SNR_dB/20); % Complex Gaussian noise
        X = x + noise;                                                               % Noisy received signal matrix
        
        Rxx = (X*X')/n_snapshots;               % Sample covariance matrix of received data
        [Evec, Eval] = eig(Rxx);                % Eigen-decomposition of covariance matrix
        [~, idx] = sort(diag(Eval), 'descend'); % Sort eigenvalues in descending order
        Evec = Evec(:, idx);                    % Sort eigenvectors accordingly
        En = Evec(:, n_sources+1:end);          % Extract noise subspace (for 2 sources)

        theta_scan = -90:0.2:90;                % Angle search grid (degrees)
        Pmusic = zeros(size(theta_scan));       % Initialize MUSIC spectrum
        for t = 1:length(theta_scan)
            a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));         % Steering vector for scan angle
            Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);                   % MUSIC pseudo-spectrum value
        end

        [~, locs] = findpeaks(real(Pmusic), theta_scan, 'SortStr','descend','MinPeakDistance',2); % Find peaks in MUSIC spectrum
        locs = sort(locs(1:n_sources));           % Select top two peaks
        est_DOAs(:, mc) = locs;                   % Store estimated DOAs for this trial
    end
    
    all_est = reshape(est_DOAs, 1, []);           % Combine both sources' estimates
    std_est(n) = std(all_est);                    % Compute standard deviation for current snapshot count

    fprintf('Snapshots = %4d --> Mean1 = %.2f°, Mean2 = %.2f°, Std = %.2f°\n', ...
            n_snapshots, mean(est_DOAs(1,:)), mean(est_DOAs(2,:)), std_est(n));
end

%==================== Plotting ==============================
figure;
plot(snapshot_values, std_est, 'o-', 'LineWidth', 2);
xlabel('Number of Snapshots');
ylabel('Standard Deviation of Estimated DOA (°)');
title('MUSIC DOA Estimation Performance vs Number of Snapshots (Two Sources)');
grid on;
