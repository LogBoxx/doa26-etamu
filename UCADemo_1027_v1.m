% ======================================================================= %
% Script:   UCADemo_1027_v1
% Function: Version one is first attempt at live 4 element UCA azimuth 
%           estimation
% Author:   Gabagool
% Date:     10/27/2025
% ======================================================================= %

clear; clf

% =========== FMComms5 INITIALIZATION ========== %

x_direct = adi.FMComms5.Rx('uri','ip:192.168.0.1');
x_direct.EnabledChannels = [1 2 3 4];
x_direct.SamplesPerFrame = 200;

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

theta_scan = 0:1:360;
phi_scan = 20;

% ================= SIMULATION START ================ %

while true
    rx = x_direct();
    rx = rx';
    theta_est_directional = music_algorithm_v2(rx, D, k, ant_pos, theta_scan, phi_scan, 'isotropic')
    % r_polar = ones(size(theta_est_directional));
    % polarplot(theta_est_directional,r_polar,'bo','MarkerFaceColor', 'r')
    pause(0.2)
end
