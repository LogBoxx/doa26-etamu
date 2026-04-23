% ======================================================================= %
% Script:   FullDemo_0401_v8
% Function: Version seven is ULA/ULA MUSIC w/ rangefinding integration
% Notes:    (1) MUSIC Algorithm separate; includes FB averaging
%           (2) Rangefinder is outdated, talk with Colton about finetunes
% Author:   Parker Reeves
% Date:     04/01/2026
% ======================================================================= %

% ====================== RANGEFINDER INITIALIZATION ===================== %

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

% ==================== ARRAY INITIALIZATION (UCA) ======================= %

N = 4;                  % Number of antennas (UCA)
c = 3e8;                % Speed of light
f = 2.4e9;              % Operating Frequency
lambda = c/f;           % Operating Wavelength

% Antenna positions (UCA)
n_idx = (0:N-1)';
phi_n = 2 * pi * n_idx / N; % Angular positions of sensors

% ==================== ARRAY INITIALIZATION (ULA) ======================= %

M = 4;                   % Number of antennas (ULA)
d = 0.5;                 % Element spacing (in wavelengths)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters
J = fliplr(eye(M));

% ========================= SIGNAL RECEPTION ============================ %

Y_1 = adi.FMComms5.Rx('uri','ip:192.168.0.101');
Y_1.EnabledChannels = [1 2 3 4];

Y_2 = adi.FMComms5.Rx('uri','ip:192.168.1.101');
Y_2.EnabledChannels = [1 2 3 4];

phi_scan = -90:1:90;
theta_scan = -90:1:90;

% =========================== INITIALIZE PLOTS =========================== %

figure;

subplot(2,1,1)
h1 = plot(phi_scan, zeros(size(phi_scan)), ...
          'LineWidth', 2, 'Color', [0 0.447 0.741]);
grid on;
title('ULA MUSIC Spectrum (Direct Element Space)');
xlabel('Elevation Angle (degrees)');
ylabel('Pseudo-spectrum (dB)');
xlim([-90 90]);

subplot(2,1,2)
h2 = plot(theta_scan, zeros(size(theta_scan)), ...
          'LineWidth', 2, 'Color', [0 0.447 0.741]);
grid on;
title('ULA MUSIC Spectrum (Direct Element Space)');
xlabel('Azimuth Angle (degrees)');
ylabel('Pseudo-spectrum (dB)');
xlim([-90 90]);

% ============================== MAIN LOOP ============================== %

while true

    spectrum_1 = zeros(size(theta_scan)); % ULAAZ Array
    spectrum_2 = zeros(size(phi_scan)); % ULAEL Array

    X_1 = Y_1(); % ULAAZ Data
    X_2 = Y_2(); % ULAEL Data

    En_1 = MusicAlg(X_1,J); % ULAAZ Noise subspace
    En_2 = MusicAlg(X_2,J); % ULAEL Noise subspace

% ===================== MUSIC SPECTRUM SEARCH (ULAAZ) 1 ===================== %

    for t = 1:length(theta_scan)
        a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));
        spectrum_1(t) = 1 / abs(a_scan'*(En_1*En_1')*a_scan);
    end

    [~, idx_peak] = max(10*log10(abs(spectrum_1)));
    est_AZ = theta_scan(idx_peak);

% ===================== MUSIC SPECTRUM SEARCH (ULAEL) 2 ===================== %

    for i = 1:length(phi_scan)
        a_theta = exp(-1j*k*(0:M-1)'*delta*sind(phi_scan(i)));
        spectrum_2(i) = 1 / abs((a_theta' * (En_2 * En_2') * a_theta));
    end

    [~, max_idx] = max(10*log10(abs(spectrum_2)));
    est_EL = phi_scan(max_idx);

    [dist_cm, strength] = get_range_v1(r,est_AZ,est_EL,dev,s_az,s_el);

% =========================== VISUALIZATION ============================= %

    fprintf('Estimated Elevation: %.2f°\n', est_EL); 
    fprintf('Estimated Azimuth: %.2f°\n', est_AZ);
    fprintf('Estimated Range: %.2f°\n', dist_cm)

    set(h1, 'YData', 10*log10(abs(spectrum_2)));
    set(h2, 'YData', 10*log10(abs(spectrum_1)));

    drawnow limitrate

    pause(0.2)

end