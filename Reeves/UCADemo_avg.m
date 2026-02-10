% ======================================================================= %
% Script:   UCADemo_avg
% Function: Attempts azimuth DOA w/ temporal averaging
% Author:   Parker Reeves
% Date:     02/04/2026
% ======================================================================= %

% ======================== ARRAY INITIALIZATION ========================= %

clear; clf

rx = adi.FMComms5.Rx('uri','ip:192.168.0.101');
rx.EnabledChannels = [1 2 3 4];
rx.SamplesPerFrame = 200;

M = 4;              % # of antennas
c = 3.0e8;          % speed of light
f = 2.4e9;          % operating frequency
lambda = c/f;       % operating wavelength
k = (2*pi)/lambda;  % wave number
n_iter = 100;       % Number of trials

R = (lambda/(2*sin(pi/M)))/2;           % array radius
phi_m = (0:M-1) * (2*pi/M);    % antenna angle in radians
ant_pos = R * [cos(phi_m); sin(phi_m); zeros(1,M)]; % positions of antenna indicies; 3 rows for x, y and z

% ========================= SIGNAL INITIALIZATION ======================= %

D = 1; % number of signals
% theta_source = 0; % true azimuth
N = rx.SamplesPerFrame;
L = 20;                     % snapshots per short-time block
K = floor(N / L);           % number of blocks
n_snapshots = rx.SamplesPerFrame;

% ======================= SIMULATION INITIALIZATION ===================== %

theta_scan = 0:0.25:360;

theta_scan = deg2rad(theta_scan); % ditto

while true

    X = rx()';

    % ============================ MUSIC ALGORITHM ========================== %

    Rxx = zeros(M,M);                   % temporal averaging

    for k_blk = 1:K
        idx = (k_blk-1)*L + (1:L);
        Xk = X(:, idx);

        Rk = (Xk * Xk') / L;
        Rxx = Rxx + Rk;
    end

    Rxx = Rxx / K;

    delta = 0.02 * trace(Rxx)/M;
    Rxx = Rxx + delta * eye(M);         % end of temp avg

    % Rxx = (X*X')/n_snapshots;
    [Evec, Eval] = eig(Rxx);
    [~, idx] = sort(diag(Eval), 'descend');
    Evec = Evec(:,idx);
    En = Evec(:, 2:end);

    Pmusic = zeros(size(theta_scan));

    for t = 1:length(theta_scan)

        a_scan = exp(-1j * k * R * cos(theta_scan(t) - phi_m)).'; %/ sqrt(M);
        % a_scan = a_scan / norm(a_scan);
        Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);

    end

    % Pmusic_dB = 10*log10(real(Pmusic) / max(real(Pmusic)));

    [~, idx_peak] = max(Pmusic);
    est_rad = theta_scan(idx_peak);

    est_DOA = rad2deg(est_rad)      % final estimation

    plot(rad2deg(theta_scan),Pmusic, 'LineWidth',1.2)
    hold on;
    title('UCA MUSIC Pseudo-Spectrum')
    xlabel('Azimuth (degrees)')
    ylabel('Normalized pseudo-spectrum (dB)')
    xline(est_DOA,'--r','LineWidth',1.2)
    xlim([0,360])
    legend('Pseudo-Spectrum','Estimated Azimuth')
    grid on;
    hold off;
    pause(0.2)

end








