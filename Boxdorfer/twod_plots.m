% twod_plots.m  (standalone)
clear; clc; close all;

% ---------------- constants ----------------
c = 3e8;
f = 2.4e9;
lambda = c/f;
k = 2*pi/lambda;

% ---------------- geometry -----------------
% ULA (vertical)
N_ula = 4;
d_ula = lambda/2;
z0_ula = 0.0762;                 % ~3 in above ground
r_ula = zeros(N_ula,3);
for n = 1:N_ula
    r_ula(n,:) = [0 0 z0_ula + (n-1)*d_ula];
end

% UCA (horizontal ring at z=0.305 m), chord spacing = lambda/2
N_uca = 4;
s_chord = lambda/2;
R_uca = s_chord/(2*sin(pi/N_uca));
z_uca = 0.305;

phi_el = (0:N_uca-1)*(360/N_uca);     % 0,90,180,270
r_uca = zeros(N_uca,3);
for n = 1:N_uca
    r_uca(n,:) = [R_uca*cosd(phi_el(n)) R_uca*sind(phi_el(n)) z_uca];
end

% ---------------- cuts ---------------------
floor_dB = -70;

% UCA azimuth cut at alpha=0 (horizontal plane)
alpha_cut = 0;
phi = -180:0.1:180;

sx = cosd(alpha_cut) * cosd(phi);
sy = cosd(alpha_cut) * sind(phi);
sz = sind(alpha_cut) * ones(size(phi));
s_hat = [sx(:) sy(:) sz(:)];

AF_uca = zeros(size(phi));
for i = 1:numel(phi)
    phase = k*(r_uca*s_hat(i,:).');
    AF_uca(i) = abs(sum(exp(1j*phase)));
end

P_uca = AF_uca.^2;
P_uca = P_uca / max(P_uca);
P_uca_dB = 10*log10(P_uca + eps);
P_uca_dB = max(P_uca_dB, floor_dB);

% ULA elevation cut at phi=0 plane (alpha is elevation from horizontal)
alpha = -90:0.1:90;
phi_fix = 0;

sx2 = cosd(alpha) * cosd(phi_fix);
sy2 = cosd(alpha) * sind(phi_fix);
sz2 = sind(alpha);
s_hat2 = [sx2(:) sy2(:) sz2(:)];

AF_ula = zeros(size(alpha));
for i = 1:numel(alpha)
    phase = k*(r_ula*s_hat2(i,:).');
    AF_ula(i) = abs(sum(exp(1j*phase)));
end

P_ula = AF_ula.^2;
P_ula = P_ula / max(P_ula);
P_ula_dB = 10*log10(P_ula + eps);
P_ula_dB = max(P_ula_dB, floor_dB);

% ---------------- plot ---------------------
figure;
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

nexttile;
polarplot(deg2rad(phi), P_uca_dB, 'LineWidth', 1.6);
title(sprintf('UCA (N=%d) Azimuth AF-only (\\alpha=%d^\\circ)', N_uca, alpha_cut));
rlim([floor_dB 0]); rticks([floor_dB -40 -20 -10 0]);
thetaticks(-180:45:180); grid on;

nexttile;
polarplot(deg2rad(alpha), P_ula_dB, 'LineWidth', 1.6);
title(sprintf('ULA (N=%d) Elevation AF-only (\\phi=%d^\\circ)', N_ula, phi_fix));
rlim([floor_dB 0]); rticks([floor_dB -40 -20 -10 0]);
thetaticks(-90:30:90); grid on;
