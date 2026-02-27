% ======================================================================= %
% Script:   DOASim_1202_v3
% Function: Version three compiles UCA and ULA sim into one complete DOA
%           estimation in both azimuth and elevation planes for moving
%           source and visualizes in 3d using MUSIC_1209_vX
% Author:   Parker Reeves
% Date:     12/09/2025
% ======================================================================= %

% ======================== ARRAY INITIALIZATION ========================= %

clear;

M_c = 4;            % # of antennas (uca)
M_l = 4;            % # of antennas (ula)
c = 3.0e8;          % speed of light
f = 2.4e9;          % operating frequency
lambda = c/f;       % operating wavelength
k = (2*pi)/lambda;  % wave number

d = 0.5;                 % ULA element spacing in wavelengths
delta = d*lambda;        % Element spacing in meters

R = (lambda/(2*sin(pi/M_c)))/2;           % array radius
phi_m = (0:M_c-1) * (2*pi/M_c);    % antenna angle in radians
ant_pos = R * [cos(phi_m); sin(phi_m); zeros(1,M_c)]; % positions of antenna indicies; 3 rows for x, y and z

% ========================= SIGNAL INITIALIZATION ======================= %

D = 1; % number of signals
theta_source = 55; % true azimuth
phi_source = 42; % true elevation
N = 200; % number of snapshots



r = raspi('169.254.52.8','analog','analog');
    if ~exist('s_az', 'var')
    s_az = servo(r, 12, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);  
    end
    if ~exist('s_el', 'var')
    s_el = servo(r, 13, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);  
    end
    dev = serialdev(r, '/dev/serial0', 115200, 8, 'none', 1);
    dev.Timeout = 0.05;

% ======================= SIMULATION INITIALIZATION ===================== %

val_SNR = 0;
theta_scan = 0:0.25:360;
phi_scan = 0:0.25:90;

n_snapshots = 200;
n_loops = 50;

% ============================ SIMULATION START ========================= %

for i = 1:n_loops

    theta_source = theta_source + 3;
    phi_source = phi_source + 0.5;

    theta_source = deg2rad(theta_source); % radians for calculation 
    theta_scan = deg2rad(theta_scan); % ditto

    phi_source = deg2rad(phi_source);
    phi_scan = deg2rad(phi_scan);

% ======================== TRUE SOURCE GENERATION ======================= %

    phase_shift_uca = exp(-1j * k * R * cos(theta_source - phi_m)).';              % UCA Steering vector for true DOA
    phase_shift_ula = exp(-1j * k * (0:M_l-1)' *delta* sin(phi_source));            % ULA Steering vector for true DOA

    s = randn(1, n_snapshots) + 1j*randn(1, n_snapshots)/sqrt(2);                % Generate random complex source signal
    noise = (randn(M_c, n_snapshots) + 1j*randn(M_c, n_snapshots))/sqrt(2); % Complex Gaussian noise
    noise = noise * 10^(-val_SNR/20);

    X_uca = (phase_shift_uca * s) + noise;    
    X_ula = (phase_shift_ula * s) + noise;

% ============================ MUSIC ALGORITHM ========================== %

    [Pmusic_uca,est_rad_uca] = MUSIC_1209_v1(X_uca,n_snapshots,theta_scan,phi_scan,1,k,R,phi_m,delta,M_l);
    [Pmusic_ula,est_rad_ula] = MUSIC_1209_v1(X_ula,n_snapshots,theta_scan,phi_scan,0,k,R,phi_m,delta,M_l);

    est_DOA_uca = rad2deg(est_rad_uca);      % final azimuth estimation
    est_DOA_ula = rad2deg(est_rad_ula);      % final elevation estimation

% ============================= PLOTTING ================================ %

    theta_source = rad2deg(theta_source); 
    theta_scan = rad2deg(theta_scan);
    phi_source = rad2deg(phi_source);
    phi_scan = rad2deg(phi_scan);

    Paz = Pmusic_uca / max(Pmusic_uca);
    Pel = Pmusic_ula / max(Pmusic_ula); % 3-dimensional computations
    
    Paz = Paz(:).';
    Pel = Pel(:); % more 3-dimensional computations

    Pjoint = Pel * Paz; % even more

    [AZ, EL] = meshgrid(theta_scan, phi_scan); % and more

    subplot(2,2,1)
    plot(theta_scan,Pmusic_uca)
    xlabel('Azimuth (Degrees)')
    ylabel('Pseudo-Spectrum (dB)')
    title('UCA Pseudo-Spectrum')
    xline(est_DOA_uca,'--g')
    xline(theta_source,'--r')
    legend('Spectrum','Estimate','True')
    xlim([0,360])

    subplot(2,2,2)
    plot(phi_scan,Pmusic_ula)
    xlabel('Elevation (Degrees)')
    ylabel('Pseudo-Spectrum (dB)')
    title('ULA Pseudo-Spectrum')
    xline(est_DOA_ula,'--g')
    xline(phi_source,'--r')
    legend('Spectrum','Estimate','True')
    xlim([0,90])

    subplot(2,1,2)
    surf(AZ,EL,Pjoint,'EdgeColor','None')
    xlabel('Azimuth (Degrees)')
    ylabel('Elevation (Degrees)')
    zlabel('Pseudo-Spectrum (dB)')
    ylim([0,90])
    xlim([0,360])

    pause(0.02)
    range = parfeval(backgroundPool,@get_range_v3,2,r,AZ,EL,s_az,s_el);
end
   



    
    
