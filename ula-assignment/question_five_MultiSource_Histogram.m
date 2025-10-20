clc; clear; close all;% Script    question_four_StdDev_v_Snapshots
% Script    question_five_MultiSource_Histogram
% Purpose  	Calculate the Histogram for Mulitple Sources across 300 MC trials
%               for a Uniform Linear Array
% Notes     (1) Number on Antennas: 10
%           (2) SNR = 10dB
%           (3) Number of Snapshots: 200
%           (4) Source Location: 60, 75 & 85 deg
% Author    2026 DoA Team
% Date     	10/6/2025

clc; clear; close all;

M = 10;                             % Number of array elements
d = 0.5;                            % Element spacing (in wavelengths)
theta_true = [60 75 85];            % True source directions (DOAs) in degrees
SNR_dB = 10;                        % Fixed SNR (dB)
n_snapshots = 200;                  % Number of temporal samples
n_trials = 300;                     % Number of Monte Carlo trials
c = 3e8; f = 2.4e9;                 % Speed of light and frequency
lambda = c/f;                       % Wavelength (m)
k = 2*pi/lambda;                    % Wave number (rad/m)
delta = d*lambda;                   % Element spacing (m)
n_sources = length(theta_true);     % Number of sources

est_all = zeros(n_sources, n_trials);                   % Store estimated DOAs for each source

for mc = 1:n_trials
    A = zeros(M, n_sources);
    for s = 1:n_sources
        A(:, s) = exp(-1j*k*(0:M-1)'*delta*sind(theta_true(s)));            % Steering vector for each source
    end
    
    s_sig = randn(n_sources, n_snapshots) + 1j*randn(n_sources, n_snapshots);
    
    X = A*s_sig; 
    noise = (randn(M, n_snapshots) + 1j*randn(M, n_snapshots))/sqrt(2)*10^(-SNR_dB/20);
    X = X + noise;
    
    Rxx = (X*X')/n_snapshots;
    [Evec, Eval] = eig(Rxx);
    [~, idx] = sort(diag(Eval), 'descend');
    Evec = Evec(:, idx);
    En = Evec(:, n_sources+1:end);
    
    theta_scan = 0:0.01:90;
    Pmusic = zeros(size(theta_scan));
    for t = 1:length(theta_scan)
        a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));
        Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);
    end
    
    [~, locs] = findpeaks(real(Pmusic), theta_scan, ...
                          'SortStr','descend','MinPeakDistance',1);
    locs = sort(locs(1:n_sources));                 % Take top n_sources peaks
    est_all(:, mc) = locs;                          % Store estimated DOAs
end

%==================== Plotting ==============================
figure;
hold on;
edges = 50:0.1:90;
colors = ['b' 'g' 'm'];

for s = 1:n_sources
    histogram(est_all(s, :), edges, 'FaceAlpha', 0.6, 'FaceColor', colors(s));
end

xline(theta_true(1), '--b', 'LineWidth', 2);
xline(theta_true(2), '--g', 'LineWidth', 2);
xline(theta_true(3), '--m', 'LineWidth', 2);
xlabel('Estimated DOA (°)');
ylabel('Count');
title('MUSIC Estimated DOA Histogram for Multiple Sources (Finer Scan)');
legend('60° Source','75° Source','85° Source','True 60°','True 75°','True 85°');
grid on;
hold off;


