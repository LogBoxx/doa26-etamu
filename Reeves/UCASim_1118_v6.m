% ======================================================================= %
% Script:   UCASim_1118_v6
% Function: Version six performs statistical analysis of up to date UCA sim
% Author:   Parker Reeves
% Date:     12/02/2025
% ======================================================================= %

% ======================== ARRAY INITIALIZATION ========================= %

clear;

M = 4;              % # of antennas
c = 3.0e8;          % speed of light
f = 2.4e9;          % operating frequency
lambda = c/f;       % operating wavelength
k = (2*pi)/lambda;  % wave number

R = (lambda/(2*sin(pi/M)))/2;           % array radius
phi_m = (0:M-1) * (2*pi/M);    % antenna angle in radians
ant_pos = R * [cos(phi_m); sin(phi_m); zeros(1,M)]; % positions of antenna indicies; 3 rows for x, y and z

% ========================= SIGNAL INITIALIZATION ======================= %

D = 1; % number of signals
% theta_source = 0; % true azimuth
N = 200; % number of snapshots

% ======================= SIMULATION INITIALIZATION ===================== %

val_SNR = 10;
theta_scan = 0:0.25:360;

theta_scan = deg2rad(theta_scan); % ditto
n_snapshots = 200;
n_loops = 360;
n_iter = 100;

% ============================ SIMULATION START ========================= %

pct_dev = zeros(size(n_loops));
est_DOAs = zeros(size(n_loops));
avg_dev = zeros(size(n_iter));

for jpeg = 1:n_iter

    theta_source = 0;
    pct_dev = zeros(size(n_loops));

for epoch = 1:n_loops

    theta_source = theta_source + 1;
    theta_source = deg2rad(theta_source);

% ======================== TRUE SOURCE GENERATION ======================= %

    phase_shift = exp(-1j * k * R * cos(theta_source - phi_m)).'; %/ sqrt(M);  % Steering vector for true DOA
    s = randn(1, n_snapshots) + 1j*randn(1, n_snapshots)/sqrt(2);                % Generate random complex source signal
    noise = (randn(M, n_snapshots) + 1j*randn(M, n_snapshots))/sqrt(2); % Complex Gaussian noise
    noise = noise * 10^(-val_SNR/20);

    X = (phase_shift * s) + noise;       

% ============================ MUSIC ALGORITHM ========================== %

    Rxx = (X*X')/n_snapshots;

    [Evec, Eval] = eig(Rxx);
    [~, idx] = sort(diag(Eval), 'descend');
    Evec = Evec(:,idx);
    En = Evec(:, 2:end);

    Pmusic = zeros(size(theta_scan));

    for t = 1:length(theta_scan)

        a_scan = exp(-1j * k * R * cos(theta_scan(t) - phi_m)).'; %/ sqrt(M);
        a_scan = a_scan / norm(a_scan);
        Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);

    end

    Pmusic_dB = 10*log10(real(Pmusic) / max(real(Pmusic)));

    [~, idx_peak] = max(Pmusic);
    est_rad = theta_scan(idx_peak);

    est_DOA = rad2deg(est_rad);      % final estimation
    est_DOAs(epoch) = est_DOA;

    plot(rad2deg(theta_scan),Pmusic_dB, 'LineWidth',1.2)
    hold on;
    title('UCA MUSIC Pseudo-Spectrum')
    xlabel('Azimuth (degrees)')
    ylabel('Normalized pseudo-spectrum (dB)')
    xline(est_DOA,'--r','LineWidth',1.2)
    xline(rad2deg(theta_source),':g','LineWidth',1.2)
    xlim([0,360])
    ylim([-25,1])
    legend('Pseudo-Spectrum','Estimated Azimuth','True Azimuth')
    grid on;

    hold off;

    theta_source = rad2deg(theta_source);
    pct_dev(epoch) = ((est_DOA - theta_source) / theta_source) * 100;
    
    pause(0.01)

end

avg_dev(jpeg) = mean(pct_dev);

end

tot_avg_dev = mean(avg_dev)

   



    
    
