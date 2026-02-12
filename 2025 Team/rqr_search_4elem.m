function [az_DOA_deg, spatialSpectrum_dB, xaxis] = rqr_search_4elem(rx_m, num_sources, dtheta_deg)
% rqr_search_4elem performs azimuth-only DOA estimation using a 
% Rank-Revealing QR and a spectral search technique for 4-element arrays.
%
% Inputs:
%   rx_m        - Received data matrix (4 x N snapshots) from the 4-element ULA.
%   num_sources - Number of source signals to estimate.
%   dtheta_deg  - Angular resolution (in degrees) for the spectral search.
%
% Outputs:
%   az_DOA_deg         - Estimated azimuth DOA(s) in degrees (sorted).
%   spatialSpectrum_dB - The computed spatial spectrum (in dB) over the search grid.
%   xaxis              - The angular grid (in degrees) over which the spectrum is computed.
%
% The method:
%   1. Estimates the covariance matrix Ryy = (rx_m*rx_m')/N.
%   2. Performs a QR decomposition of Ryy (which reveals the signal subspace rank).
%   3. Partitions R into R11 and R12 and forms the noise subspace basis:
%         Psi = [ -inv(R11)*R12; I_{(M-num_sources)} ]
%   4. Computes the orthogonal projection onto this null space:
%         Psi_O = Psi * inv(Psi' * Psi) * Psi'
%   5. Scans over candidate angles and computes the spatial spectrum:
%         P(theta) = 1/(a(theta)' * Psi_O * a(theta))
%   6. Peaks in the spectrum correspond to the DOA estimates.

    %% Parameters
    [M, N] = size(rx_m);  % M should be 4 for our ULA
    if M ~= 4
        error('This function is designed for 4-element arrays only.');
    end
    
    d = 0.5;  % Element spacing (in wavelengths)
    
    % Define physical element indices for a 4-element ULA:
    % For M = 4, we use: -2, -1, 0, +1
    arrayIndices = (-M/2 : M/2 - 1).';  % column vector of size 4x1

    %% Step 1: Covariance Matrix Estimation
    Ryy = (rx_m * rx_m') / N;  % 4x4 covariance matrix

    %% Step 2: QR Decomposition (RRQR)
    % Perform QR decomposition on Ryy
    [~, R] = qr(Ryy);
    
    %% Step 3: Partition R and Form the Nullspace Basis
    % Partition R into R11 (num_sources x num_sources) and R12 (num_sources x (M-num_sources))
    R11 = R(1:num_sources, 1:num_sources);
    R12 = R(1:num_sources, num_sources+1:end);
    
    % Form the nullspace basis Psi:
    % Psi = [ -inv(R11)*R12; I_{(M-num_sources)} ]
    Psi = [ -inv(R11)*R12; eye(M - num_sources) ];
    
    %% Step 4: Orthogonal Projection onto the Noise Subspace
    % Compute the projection matrix onto the null space:
    Psi_O = Psi * inv(Psi' * Psi) * Psi';
    
    %% Step 5: Spatial Spectrum Computation
    % Define the search grid for azimuth angles (in degrees)
    xaxis = -90:dtheta_deg:90;  
    P = zeros(1, length(xaxis));
    
    % For each candidate angle, compute the steering vector and then the spectrum
    for idx = 1:length(xaxis)
        theta_deg = xaxis(idx);
        theta_rad = theta_deg * pi/180;
        % Steering vector for the 4-element ULA:
        % a(theta) = exp(j*2*pi*d * arrayIndices * sin(theta))
        a = exp(1j * 2*pi*d * arrayIndices * sin(theta_rad));
        % Compute denominator: a' * Psi_O * a
        denom = a' * Psi_O * a;
        % Spatial spectrum:
        P(idx) = 1 / abs(denom);
    end
    
    % Normalize and convert the spectrum to dB scale.
    spatialSpectrum_dB = 10*log10(P / max(P));
    
    %% Step 6: DOA Estimation via Peak Search
    % For a single source, simply take the maximum.
    % For multiple sources, use findpeaks to extract the largest num_sources peaks.
    if num_sources == 1
        [~, peakIndex] = max(P);
        az_DOA_deg = xaxis(peakIndex);
    else
        [pks, locs] = findpeaks(P, 'SortStr','descend');
        num_found = min(num_sources, length(locs));
        if num_found < num_sources
            warning('Found only %d peaks when %d sources were requested.', num_found, num_sources);
            % If fewer peaks found, use the highest points instead
            if isempty(locs)
                [~, sorted_indices] = sort(P, 'descend');
                locs = sorted_indices(1:min(num_sources, length(sorted_indices)));
            end
        end
        az_DOA_deg = sort(xaxis(locs(1:num_found)));
    end
end
