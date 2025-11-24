% ======================================================================= %
% Script:   UCASim_1118_v5
% Function: Version five performs all calculations in one script (azimuth
%           only)
% Author:   Edgar Allan Poe
% Date:     11/18/2025
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
theta_source = 284; % true azimuth
N = 200; % number of snapshots

% ======================= SIMULATION INITIALIZATION ===================== %

val_SNR = 0;
theta_scan = 0:0.25:360;

theta_source = deg2rad(theta_source); % radians for calculation 
theta_scan = deg2rad(theta_scan); % ditto
n_snapshots = 200;

% ============================ SIMULATION START ========================= %

while true  % loop for continued estimations

% ======================== TRUE SOURCE GENERATION ======================= %

    phase_shift = exp(-1j * k * R * cos(theta_source - phi_m)).'; %/ sqrt(M);  % Steering vector for true DOA
    s = randn(1, n_snapshots) + 1j*randn(1, n_snapshots)/sqrt(2);                % Generate random complex source signal
    noise = (randn(M, n_snapshots) + 1j*randn(M, n_snapshots))/sqrt(2); % Complex Gaussian noise
    noise = noise * 10^(-val_SNR/20);

    X = (phase_shift * s) + noise;       

% ============================ MUSIC ALGORITHM ========================== %

    Rxx = (X*X')/n_snapshots;
    %Rfb = Rxx + flip(eye(M)*conj(Rxx)*flip(eye(M)));

    [Evec, Eval] = eig(Rxx);
    [~, idx] = sort(diag(Eval), 'descend');
    Evec = Evec(:,idx);
    En = Evec(:, 2:end);

    Pmusic = zeros(size(theta_scan));

    for t = 1:length(theta_scan)
        a_scan = exp(-1j * k * R * cos(theta_scan(t) - phi_m)).'; %/ sqrt(M);

        Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);

    end

    [~, idx_peak] = max(Pmusic);
    est_rad = theta_scan(idx_peak);

    est_DOA = rad2deg(est_rad)      % final estimation

    pause(2)

end
   



    
    
