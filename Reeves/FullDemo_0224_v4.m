% ======================================================================= %
% Script:   FullDemo_0224_v2
% Function: Version one attempts full DOA spectrum in both azimuth and
%           elevation planes utilizing pure MUSIC algorithm
% Notes:    (1) MUSIC Algorithm separate; includes FB averaging
%           (2) Range finding algorithm implemented
% Author:   Parker Reeves
% Date:     02/24/2026
% ======================================================================= %

% ==================== ARRAY INITIALIZATION (UCA) ======================= %

N = 4;                  % Number of antennas (UCA)
c = 3e8;                % Speed of light
f = 2.4e9;              % Operating Frequency
lambda = c/f;           % Operating Wavelength

R_radius = lambda/(2*sqrt(2)); % Radius of the circular array !!!
prev_DOA = 0; % Initialize resolveAmbiguity function
J_1 = [0 0 1 0; 0 0 0 1; 1 0 0 0; 0 1 0 0]; % FB exchange matrix 

% Antenna positions (UCA)
n_idx = (0:N-1)';
phi_n = 2 * pi * n_idx / N; % Angular positions of sensors

% ==================== ARRAY INITIALIZATION (ULA) ======================= %

M = 4;                   % Number of antennas (ULA)
d = 0.5;                 % Element spacing (in wavelengths)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters
J_2 = fliplr(eye(M));    % FB exchange matrix

% ======================== LIDAR INITIALIZATION ========================= %

clear r;
r = raspi('169.254.52.8','analog','analog');
if ~exist('s_az', 'var')
    s_az = servo(r, 12, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
end
if ~exist('s_el', 'var')
    s_el = servo(r, 13, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
end
dev = serialdev(r, '/dev/serial0', 115200, 8, 'none', 1);
dev.Timeout = 0.05;

% ========================= SIGNAL RECEPTION ============================ %

Y_1 = adi.FMComms5.Rx('uri','ip:192.168.0.101');
Y_1.EnabledChannels = [1 2 3 4];

Y_2 = adi.FMComms5.Rx('uri','ip:192.168.1.101');
Y_2.EnabledChannels = [1 2 3 4];

theta_scan = 0:0.1:359.9;
phi_scan = -90:1:90;

% =========================== INITIALIZE PLOTS =========================== %

figure;

subplot(2,1,1)
h1 = plot(theta_scan, zeros(size(theta_scan)), ...
          'LineWidth', 2, 'Color', [0 0.447 0.741]);
grid on;
title('UCA MUSIC Spectrum (Direct Element Space)');
xlabel('Azimuth Angle (degrees)');
ylabel('Pseudo-spectrum (dB)');
xlim([0 360]);

subplot(2,1,2)
h2 = plot(phi_scan, zeros(size(phi_scan)), ...
          'LineWidth', 2, 'Color', [0 0.447 0.741]);
grid on;
title('ULA MUSIC Spectrum (Direct Element Space)');
xlabel('Azimuth Angle (degrees)');
ylabel('Pseudo-spectrum (dB)');
xlim([-90 90]);

% ============================== MAIN LOOP ============================== %

while true

    spectrum_2 = zeros(size(phi_scan));
    spectrum_1 = zeros(size(theta_scan));

    X_1 = Y_1(); % UCA Data
    X_2 = Y_2(); % ULA Data

    En_1 = MusicAlg(X_1,J_1); % UCA Noise subspace
    En_2 = MusicAlg(X_2,J_2);

% ===================== MUSIC SPECTRUM SEARCH (UCA) ===================== %

    for i = 1:length(theta_scan)
        a_theta = exp(1j * 2 * pi * R_radius / lambda * ...
                      cosd(theta_scan(i) - rad2deg(phi_n)));
        spectrum_1(i) = 1 / abs((a_theta' * (En_1 * En_1') * a_theta));
    end

    [~, max_idx] = max(10*log10(abs(spectrum_1)));
    est_AZ = theta_scan(max_idx);
    %est_AZ = resolveAmbiguity(est_DOA, prev_DOA);

% ===================== MUSIC SPECTRUM SEARCH (ULA) ===================== %

    for t = 1:length(phi_scan)
        a_scan = exp(-1j*k*(0:M-1)'*delta*sind(phi_scan(t)));
        spectrum_2(t) = 1 / abs(a_scan'*(En_2*En_2')*a_scan);
    end

    [~, idx_peak] = max(10*log10(abs(spectrum_2)));
    est_EL = phi_scan(idx_peak);

% =========================== RANGE FINDER ============================== %

    get_range_v1(r,est_AZ,est_EL,dev,s_az,s_el);

% =========================== VISUALIZATION ============================= %

    fprintf('Estimated Azimuth: %.2f°\n', est_AZ); 
    fprintf('Estimated Elevation: %.2f°\n', est_EL);

    set(h1, 'YData', 10*log10(abs(spectrum_1)));
    set(h2, 'YData', 10*log10(abs(spectrum_2)));

    drawnow limitrate

    %prev_DOA = est_AZ;

    pause(0.2)

end