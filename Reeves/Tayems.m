% MUSIC for 4-Antenna UCA (Direct Element Space)
clear; clc;

%% 1. Simulation Setup
N = 4;                  % Number of antennas
theta_true = 220;        % True Azimuth Angle (degrees)
SNR = 10;               % Signal-to-Noise Ratio
K = 300;                % Snapshots
lambda = 1;             % Wavelength
R_radius = 0.5 * lambda;% Radius of the circular array

% Antenna positions (UCA)
n_idx = (0:N-1)';
phi_n = 2 * pi * n_idx / N; % Angular positions of sensors

%% 2. Signal Generation
% % Steering vector for UCA (Non-linear phase)
% % a(theta) = exp(j * 2*pi*R/lambda * cos(theta - phi_n))
% a = exp(1j * 2 * pi * R_radius / lambda * cos(deg2rad(theta_true) - phi_n));

% Received Signal
% s = (randn(1, K) + 1j * randn(1, K)) / sqrt(2);
% noise = (randn(N, K) + 1j * randn(N, K)) * 10^(-SNR/20) / sqrt(2);
Y = adi.FMComms5.Rx('uri','ip:192.168.0.101');
Y.EnabledChannels = [1 2 3 4];

while true

    X = Y()';

    %% 3. Covariance and Subspace Decomposition
    R_x = (X * X') / K;     % Sample Covariance Matrix
    [U, D] = eig(R_x);      % Eigen-decomposition
    [~, idx] = sort(diag(D), 'descend');
    U = U(:, idx);

    % Noise subspace (For 1 source, columns 2 to N)
    En = U(:, 2:end);

    %% 4. MUSIC Spectrum Search
    angles = 0:0.1:360;
    spectrum = zeros(size(angles));

    for i = 1:length(angles)
        theta_test = deg2rad(angles(i));
        % Steering vector for the search angle
        a_theta = exp(1j * 2 * pi * R_radius / lambda * cos(theta_test - phi_n));

        % MUSIC pseudo-spectrum formula
        spectrum(i) = 1 / (a_theta' * (En * En') * a_theta);
    end

    %% 5. Visualization
    [~, max_idx] = max(10*log10(abs(spectrum)));
    fprintf('Estimated DOA: %.2f°\n', angles(max_idx));

    plot(angles, 10*log10(abs(spectrum)), 'LineWidth', 2, 'Color', [0 0.447 0.741]);
    grid on; hold on;
    title('UCA MUSIC Spectrum (Direct Element Space)');
    xlabel('Azimuth Angle (degrees)');
    ylabel('Pseudo-spectrum (dB)');
    xlim([0 360]);
    hold off

    pause(0.2)

end