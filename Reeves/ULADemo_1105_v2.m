% Script    ULADemo_1105_v2
% Purpose  	V2 attempts ULA estimation (with filtering)
% Notes     
% Author    John Direction of Arrival
% Date     	11/05/2025

clc; clear; close all;

x_direct = adi.FMComms5.Rx('uri','ip:192.168.0.1');
x_direct.EnabledChannels = [1 2 3 4];

n_snapshots = x_direct.SamplesPerFrame;
Fs = x_direct.SamplingRate;

M = 4;                   % Number of array elements
d = 0.5;                 % Element spacing (in wavelengths)
c = 3e8; f = 2.4e9;      % Speed of light and operating frequency
lambda = c/f;            % Wavelength (m)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters

theta_scan = -90:0.2:90;                % Angle search grid (degrees)

while true
    X = x_direct();                         % Noisy received signal matrix
    X = flip(X');

    X = lowpass(X,60e3,Fs);
    
    Rxx = (X*X') / n_snapshots;               % Sample covariance matrix of received data
    
    Rfb = Rxx + flip(eye(M))*conj(Rxx)*flip(eye(M)); % Something Tayem Said

    [Evec, Eval] = eig(Rfb);                % Eigen-decomposition of covariance matrix
    [~, idx] = sort(diag(Eval), 'descend'); % Sort eigenvalues in descending order
    Evec = Evec(:, idx);                    % Sort eigenvectors accordingly
    En = Evec(:, 2:end);                    % Extract noise subspace (for 1 source)
    
    Pmusic = zeros(size(theta_scan));       % Initialize MUSIC spectrum
    

    for t = 1:length(theta_scan)
        a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));              % Steering vector for scan angle
        Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);                        % MUSIC pseudo-spectrum value
        [~, idx_peak] = max(Pmusic);            % Find peak index in MUSIC spectrum
        est_DOAs = theta_scan(idx_peak)         % Store estimated DOA for this trial
    end
    plot(theta_scan,Pmusic)
    hold on
    title('Music Pseudo-Spectrum')
    xlabel('Theta (Degrees)')
    ylabel('Magnitude')
    xline(est_DOAs)
    hold off
    pause(0.2)
end