% ======================================================================= %
% Script:   UCADemo_1104_v2
% Function: Version two attempts live 4 element UCA azimuth estimation with
%           filtering
% Author:   John Pork
% Date:     11/04/2025
% ======================================================================= %

clear; clf

% =========== FMComms5 INITIALIZATION ========== %

x_direct = adi.FMComms5.Rx('uri','ip:192.168.0.1');
x_direct.EnabledChannels = [1 2 3 4];
x_direct.SamplesPerFrame = 200;
Fs = x_direct.SamplingRate;

% =============== ARRAY INITIALIZATION ============== %

M = 4; % # of antennas
c = 3.0e8; % speed of light
f = 2.4e9; % frequency of transmission
lambda = c/f; % wavelength of transmission
k = (2*pi)/lambda;

R = lambda/(2*sin(pi/M)); % radius of antenna array
ant_angles = (0:M-1) * (2*pi/M); 
ant_pos = R * [cos(ant_angles); sin(ant_angles); zeros(1, M)]; % positions of antenna indicies; 3 rows for x, y and z

D = 1; % number of signals

theta_scan = 0:2.5:360;
phi_scan = 20;

% ================= SIMULATION START ================ %

while true
    rx = x_direct();
    rx_filtered = bandpass(rx,[1,2e3],Fs);
    rx_filtered = rx_filtered';
    theta_est_directional = music_algorithm_v2(rx_filtered, D, k, ant_pos, theta_scan, phi_scan, 'isotropic')
    pause(0.2)
end
