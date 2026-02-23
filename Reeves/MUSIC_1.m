clear; close all; clc;

% Parameters
N = 10;                 % number of array elements
d = 0.5;                % inter-element spacing (in wavelengths)
K = 2;                  % number of sources
true_DOAs = [20 25];    % true source directions in degrees
Nsnap = 50;            % number of snapshots
SNR_dB = 10;            % SNR per source

% Array steering vector
steer = @(theta_deg) exp(-1j*2*pi*d*(0:N-1)'*sin(theta_deg*pi/180));

% Generate source signals and noise
S = (randn(K, Nsnap) + 1j*randn(K, Nsnap))/sqrt(2); % K x Nsnap (unit power)

% Steering matrix for the two sources
A = [steer(true_DOAs(1)), steer(true_DOAs(2))]; % N x K

% Noise
signal_power = 1;
SNR = 10^(SNR_dB/10);
noise_variance = signal_power / SNR;
Noise = sqrt(noise_variance/2)*(randn(N, Nsnap) + 1j*randn(N, Nsnap));

% Received data
X = A*S + Noise; % N x Nsnap

% Sample covariance matrix
Rxx = (1/Nsnap) * (X * X');

% Eigendecomposition
[V,D] = eig(Rxx);
[eigvals, idx] = sort(real(diag(D)), 'descend');
V = V(:, idx);

% Noise subspace
Un = V(:, K+1:end); % N x (N-K)

% MUSIC pseudospectrum
theta_scan = -90:0.1:90;
P = zeros(size(theta_scan));
for ii = 1:length(theta_scan)
    a = steer(theta_scan(ii));
    P(ii) = 1 / real( a' * (Un * Un') * a );
end

% Normalize to dB
P = abs(P);
P_dB = 10*log10(P / max(P));

% Find top-K peaks
[~, locs] = findpeaks(P, theta_scan, 'SortStr','descend');
est_DOAs = sort(locs(1:K));

fprintf('True DOAs = [%.2f, %.2f] deg\n', true_DOAs(1), true_DOAs(2));
fprintf('Estimated DOAs (MUSIC) = [%.2f, %.2f] deg\n', est_DOAs(1), est_DOAs(2));

% Plot MUSIC pseudospectrum
figure('Color','w','Position',[200 200 700 420]);
plot(theta_scan, P_dB, 'LineWidth',1.5);
hold on;
plot(true_DOAs, interp1(theta_scan, P_dB, true_DOAs), 'rv','MarkerFaceColor','r','MarkerSize',8)
plot(est_DOAs, interp1(theta_scan, P_dB, est_DOAs), 'kp','MarkerFaceColor','k','MarkerSize',10);
grid on;
xlabel('Angle (degrees)');
ylabel('Normalized MUSIC Spectrum (dB)');
title(sprintf('MUSIC Spectrum (N=%d, Snapshots=%d, SNR=%.1f dB)', N, Nsnap, SNR_dB));
legend('MUSIC spectrum','True DOAs','Estimated DOAs','Location','SouthEast');
xlim([-90 90]);
ylim([max(P_dB)-50 max(P_dB)+1]);
