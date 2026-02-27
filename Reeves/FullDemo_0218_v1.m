% ======================================================================= %
% Script:   FullDemo_0218_v1
% Function: Version one attempts full DOA spectrum in both azimuth and
%           elevation planes utilizing pure MUSIC algorithm
% Notes:    (1) Script designed for static estimation
% Author:   Parker Reeves
% Date:     02/18/2026
% ======================================================================= %

% ==================== ARRAY INITIALIZATION (UCA) ======================= %

N = 4;                  % Number of antennas (UCA)
c = 3e8;                % Speed of light
f = 2.4e9;              % Operating Frequency
lambda = c/f;           % Operating Wavelength

R_radius = 0.3   * lambda; % Radius of the circular array !!!
prev_DOA = 0; % Inizialize resolveAmbiguity function

% Antenna positions (UCA)
n_idx = (0:N-1)';
phi_n = 2 * pi * n_idx / N; % Angular positions of sensors

% ==================== ARRAY INITIALIZATION (ULA) ======================= %

M = 4;                   % Number of antennas (ULA)
d = 0.5;                 % Element spacing (in wavelengths)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters

% ========================= SIGNAL RECEPTION ============================ %

Y_1 = adi.FMComms5.Rx('uri','ip:192.168.0.101');
Y_1.EnabledChannels = [1 2 3 4];

Y_2 = adi.FMComms5.Rx('uri','ip:192.168.1.101');
Y_2.EnabledChannels = [1 2 3 4];

theta_scan = 0:0.1:359.9;
phi_scan = -90:1:90;                % Angle search grid (degrees)

while true

    spectrum_2 = zeros(size(phi_scan));
    spectrum_1 = zeros(size(theta_scan));

    X_1 = Y_1();
    X_1 = X_1';             % Correction to IQ matrix dimensions

    X_2 = Y_2();
    X_2 = X_2';

    R_x_1 = (X_1 * X_1') / size(X_1,2);
    R_x_2 = (X_2 * X_2') / size(X_2,2);

    % FB averaging (UCA)
    J = [0 0 1 0; 0 0 0 1; 1 0 0 0; 0 1 0 0];
    R_fb_1 = 0.5 * (R_x_1 + J * conj(R_x_1) * J);

    % FB averaging (ULA)
    J = fliplr(eye(M));
    R_fb_2 = 0.5 * (R_x_2 + J * conj(R_x_2) * J);

    [U, D] = eig(R_fb_1);      % Eigen-decomposition
    [~, idx] = sort(diag(D), 'descend');
    U_1 = U(:, idx);

    % Noise subspace (For 1 source, columns 2 to N)
    En_1 = U(:, 2:end);

    [Evec, Eval] = eig(R_fb_2);                % Eigen-decomposition of covariance matrix
    [~, idx] = sort(diag(Eval), 'descend'); % Sort eigenvalues in descending order
    Evec = Evec(:, idx);                    % Sort eigenvectors accordingly
    En_2 = Evec(:, 2:end);                    % Extract noise subspace (for 1 source)

% ===================== MUSIC SPECTRUM SEARCH (UCA) ===================== %

    for i = 1:length(theta_scan)
        theta_test = deg2rad(theta_scan(i));
        % Steering vector for the search angle
        a_theta = exp(1j * 2 * pi * R_radius / lambda * cos(theta_test - phi_n));

        % MUSIC pseudo-spectrum formula
        spectrum_1(i) = 1 / real((a_theta' * (En * En') * a_theta));
    end

    [~, max_idx] = max(10*log10(abs(spectrum_1)));
    est_DOA = theta_scan(max_idx);
    est_AZ = resolveAmbiguity(est_DOA, prev_DOA); % correction for 180 ambiguity flips

% ===================== MUSIC SPECTRUM SEARCH (ULA) ===================== %

    for t = 1:length(phi_scan)
        a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));              % Steering vector for scan angle
        spectrum_2(t) = 1 / real(a_scan'*(En*En')*a_scan);                        % MUSIC pseudo-spectrum value !!!REAL/ABS!!!
    end

    [~, idx_peak] = max(10*log10(spectrum_2));            % Find peak index in MUSIC spectrum
    est_EL = abs(theta_scan(idx_peak));         % Store estimated DOA for this trial

% =========================== VISUALIZATION ============================= %

    fprintf('Estimated Azimuth: %.2f°\n', est_AZ); 
    fprintf('Estimated Elevation: %.2f°\n', est_EL);

    subplot(2,1,1)
    plot(theta_scan, 10*log10(abs(spectrum_1)), 'LineWidth', 2, 'Color', [0 0.447 0.741]);
    grid on; hold on;
    title('UCA MUSIC Spectrum (Direct Element Space)');
    xlabel('Azimuth Angle (degrees)');
    ylabel('Pseudo-spectrum (dB)');
    xlim([0 360]);
    hold off

    subplot(2,1,2)
    plot(phi_scan, 10*log10(abs(spectrum_1)), 'LineWidth', 2, 'Color', [0 0.447 0.741]);
    grid on; hold on;
    title('ULA MUSIC Spectrum (Direct Element Space)');
    xlabel('Azimuth Angle (degrees)');
    ylabel('Pseudo-spectrum (dB)');
    xlim([-90 90]);
    hold off

    prev_DOA = est_AZ;

    pause(0.2)

end