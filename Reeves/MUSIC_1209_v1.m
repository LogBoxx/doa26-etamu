function [Pmusic,a] = MUSIC_1209_v1(X,N,theta_scan,phi_scan,q,k,R,phi_m,delta,M)
% MUSIC_1209_v1
% This function compiles the standard MUSIC algorithm and computes
% estimation in either UCA or ULA application
%
% Inputs:
%   X - Raw input data from FMComms5
%   N - Number of snapshots
%   theta_scan - Scanning vector for UCA (radians)
%   phi_scan - Scanning vector for ULA (radians)
%   q - binary value; 1=UCA, 2=ULA
%   k - wave number
%   R - UCA radius
%   phi_m - UCA antenna angle in rads
%   delta - ULA element spacing in meters
%   M - number of antennas in ULA
%
% Outputs:
%   a - Angle Estimation (radians)

Rxx = (X*X')/N;
[Evec, Eval] = eig(Rxx);

[~, idx] = sort(diag(Eval), 'descend');

Evec = Evec(:,idx);
En = Evec(:,2:end);

if q == 1
    Pmusic = zeros(size(theta_scan));
    for t = 1:length(theta_scan)
        a_scan = exp(-1j * k * R * cos(theta_scan(t) - phi_m)).';
        Pmusic(t) = 1 / abs(a_scan' * (En * En') * a_scan);
    end
else
    Pmusic = zeros(size(phi_scan));
    for t = 1:length(phi_scan)
        a_scan = exp(-1j * k *(0:M-1)' * delta *sin(phi_scan(t)));
        Pmusic(t) = 1 / abs(a_scan' * (En * En') * a_scan);
    end
end

[~, idx_peak] = max(Pmusic);

a = theta_scan(idx_peak);


