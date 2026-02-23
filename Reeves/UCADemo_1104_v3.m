% ======================================================================= %
% Script:   UCADemo_1104_v3
% Function: Version three attempts UCA demo (azimuth only)
% Author:   Edgar Allan Poe
% Date:     11/18/2025
% ======================================================================= %

% ======================== ARRAY INITIALIZATION ========================= %

clear; clf

x_direct = adi.FMComms5.Rx('uri','ip:192.168.0.1');
x_direct.EnabledChannels = [1 2 3 4];
x_direct.SamplesPerFrame = 200;
Fs = x_direct.SamplingRate;

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
N = 200; % number of snapshots

% ======================= SIMULATION INITIALIZATION ===================== %

val_SNR = 0;
theta_scan = 0:0.25:180;

theta_scan = deg2rad(theta_scan); % ditto
n_snapshots = x_direct.SamplesPerFrame;

% ============================ SIMULATION START ========================= %

while true  % loop for continued estimations

% ======================== TRUE SOURCE RECEPTION ======================= %

    %Y = x_direct()';
    %X = lowpass(Y,200e3,Fs);
    X = x_direct()';

% ============================ MUSIC ALGORITHM ========================== %

    Rxx = (X*X')/n_snapshots;
    Rfb = Rxx + flip(eye(M)*conj(Rxx)*flip(eye(M)));

    [Evec, Eval] = eig(Rfb);
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

    plot(rad2deg(theta_scan),Pmusic)
    xline(est_DOA)
    pause(0.2)

end
   



    
    
