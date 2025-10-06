% Script    question_eight_StdDev_v_NoA_MultiSource
% Purpose  	Calculate the Standard Deviation vs. Number of Antennas across 300 MC trials
%               for a Uniform Linear Array for Multiple Sources
% Notes     (1) Number on Antennas: 4 -> 24 counting by 2
%           (2) Number of Snapshots: 200
%           (3) Source Location: 60 & 75 deg
%           (4) SNR: 10dB
% Author    2026 DoA Team
% Date     	10/6/2025

clc; clear; close all;

theta_true = [60 75];    % True signal directions (DOAs) in degrees
n_snapshots = 200;       % Number of temporal samples per trial
SNR_dB = 10;             % Fixed SNR value (dB)
n_trials = 300;          % Number of Monte Carlo trials per antenna count
c = 3e8; f = 2.4e9;      % Speed of light and operating frequency
lambda = c/f;            % Wavelength (m)
d = 0.5;                 % Element spacing (in wavelengths)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters
M_values = 2:2:24;       % Number of antennas to test (2, 4, 6, ..., 24)
n_sources = length(theta_true);   % Number of signal sources

std_est = zeros(size(M_values));  % Estimation vector for standard deviation vs. antenna count

for m = 1:length(M_values)
    M = M_values(m);                 % Current number of antennas
    est_DOAs = zeros(n_sources, n_trials);   % Store estimated DOAs for each trial
    
    for mc = 1:n_trials
        A = zeros(M, n_sources);                                         % Initialize steering matrix
        for src = 1:n_sources
            A(:, src) = exp(-1j*k*(0:M-1)'*delta*sind(theta_true(src))); % Steering vector for each source
        end
        
        s_sig = randn(n_sources, n_snapshots) + 1j*randn(n_sources, n_snapshots);  % Generate random complex source signals
        x = A*s_sig;                                                               % Signal received across array elements
        noise = (randn(M, n_snapshots) + 1j*randn(M, n_snapshots))/sqrt(2)*10^(-SNR_dB/20); % Complex Gaussian noise
        X = x + noise;                                                             % Noisy received signal matrix
        
        Rxx = (X*X')/n_snapshots;               % Sample covariance matrix of received data
        [Evec, Eval] = eig(Rxx);                % Eigen-decomposition of covariance matrix
        [~, idx] = sort(diag(Eval), 'descend'); % Sort eigenvalues in descending order
        Evec = Evec(:, idx);                    % Sort eigenvectors accordingly
        En = Evec(:, n_sources+1:end);          % Extract noise subspace (for 2 sources)
        
        theta_scan = -90:0.2:90;                % Angle search grid (degrees)
        Pmusic = zeros(size(theta_scan));       % Initialize MUSIC spectrum
        for t = 1:length(theta_scan)
            a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));      % Steering vector for scan angle
            Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);                % MUSIC pseudo-spectrum value
        end
        
        [~, locs] = findpeaks(real(Pmusic), theta_scan, 'SortStr','descend','MinPeakDistance',2); % Find peaks in MUSIC spectrum
        locs = sort(locs(1:n_sources));          % Select two strongest peaks
        est_DOAs(:, mc) = locs;                  % Store estimated DOAs for this trial
    end
    
    all_est = reshape(est_DOAs, 1, []);          % Combine all source estimates
    std_est(m) = std(all_est);                   % Compute standard deviation for current antenna count
    
    fprintf('M = %2d --> Mean1 = %.2f°, Mean2 = %.2f°, Std = %.2f°\n', ...
            M, mean(est_DOAs(1,:)), mean(est_DOAs(2,:)), std_est(m));
end

%==================== Plotting ==============================
figure;
plot(M_values, std_est, 'o-', 'LineWidth', 2);
xlabel('Number of Antennas');
ylabel('Standard Deviation of Estimated DOA (°)');
title('MUSIC DOA Estimation Performance vs Number of Antennas (Two Sources)');
grid on;





