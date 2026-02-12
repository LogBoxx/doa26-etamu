% SJP_GET_DOA_RQR_SEARCH_AZ Direction of Arrival estimation using Rank-Revealing QR and Spectral Search.
%
% Purpose:
%   Performs azimuth-only Direction of Arrival (DOA) estimation for signals
%   impinging on an 8-element Uniform Linear Array (ULA). This method utilizes
%   a Rank-Revealing QR (RRQR) decomposition approach to estimate the noise
%   subspace, followed by a spectral search to find the DOAs.
%
% Array Assumption:
%   - Assumes an 8-element ULA (M=8).
%   - Assumes the input data `rx_m` (8xN) has rows corresponding to the ULA
%     elements ordered spatially (e.g., from element -4 to +3 relative to the array center).
%     This ordering is typically ensured by the data acquisition function (e.g., get_data_fmc5_OptB).
%   - Assumes half-wavelength spacing (d=0.5) between elements.
%
% Algorithm Steps:
%   1. Estimates the sample covariance matrix `Ryy = (rx_m * rx_m') / N`, where N
%      is the number of snapshots. Size: M x M (8x8).
%   2. Performs QR decomposition on the covariance matrix: `[~, R] = qr(Ryy)`.
%      While a true RRQR might use pivoting, this implementation assumes the
%      standard QR decomposition sufficiently orders the signal components in `R`.
%      `R` is an upper triangular matrix (8x8).
%   3. Partitions the upper triangular matrix `R` based on the expected number of sources:
%      - `R11`: Top-left `num_sources` x `num_sources` block.
%      - `R12`: Top-right `num_sources` x (M - `num_sources`) block.
%   4. Forms a basis `Psi` for the noise subspace using the partitioned blocks.
%      `Psi = [ -inv(R11) * R12; eye(M - num_sources) ]`. Size: M x (M - `num_sources`).
%      This construction relies on the signal and noise components being separated
%      by the QR decomposition. `inv(R11)` requires `R11` to be well-conditioned.
%   5. Computes the orthogonal projection matrix `Psi_O` onto the estimated noise subspace:
%      `Psi_O = Psi * inv(Psi' * Psi) * Psi'`. Size: M x M.
%   6. Defines an angular search grid `xaxis` from -90 to +90 degrees with resolution `dtheta_deg`.
%   7. Defines the steering vector `a` for the full 8-element ULA, assuming elements
%      indexed from -M/2 to M/2 - 1 (i.e., -4 to +3).
%   8. Scans through each angle `theta` in the grid:
%      a. Computes the steering vector `a(theta)` for the full array at that angle.
%      b. Calculates the spatial spectrum value using a MUSIC-like formula based on the
%         projection onto the noise subspace: `P(theta) = 1 / abs(a(theta)' * Psi_O * a(theta))`.
%         Peaks occur when `a(theta)` is orthogonal to the noise subspace.
%   9. Normalizes the spectrum `P` and converts it to decibels (`spatialSpectrum_dB`).
%  10. Finds the locations of the peaks in the spectrum `P` to determine the estimated
%      DOAs (`az_DOA_deg`). Handles finding single or multiple sources.
%
% Inputs:
%   rx_m        - Complex double matrix (8 x N) containing the snapshot data
%                 from the 8-element ULA. N is the number of snapshots.
%   num_sources - Integer, the expected number of signal sources.
%   dtheta_deg  - Double, the angular resolution in degrees for the spectral search grid.
%
% Outputs:
%   az_DOA_deg        - Double row vector containing the estimated azimuth DOA(s) in degrees,
%                       sorted ascendingly. Length is <= `num_sources`.
%   spatialSpectrum_dB - Double row vector containing the computed spatial spectrum
%                       in decibels (dB), normalized to a maximum of 0 dB.
%   xaxis             - Double row vector, the angular grid (in degrees) corresponding
%                       to `spatialSpectrum_dB`.
%
% Dependencies: None (uses standard MATLAB functions).
%
% Example:
%   % Assume rx_data is 8x15 complex matrix
%   [doa_estimates, spectrum, angles] = sjp_get_doa_RQR_search_az(rx_data, 1, 0.1)
function [az_DOA_deg, spatialSpectrum_dB, xaxis] = sjp_get_doa_RQR_search_az(rx_m, num_sources, dtheta_deg)


    %% Parameters
    [M, N] = size(rx_m);  % M should be 8 for our ULA
    d = 0.5;             % Element spacing (in wavelengths)
    
    % Define physical element indices assuming a symmetric ULA:
    % For M = 8, we use: -4, -3, -2, -1, 0, +1, +2, +3.
    arrayIndices = (-M/2 : M/2 - 1).';  % column vector of size 8x1

    %% Step 1: Covariance Matrix Estimation
    Ryy = (rx_m * rx_m') / N;  % 8x8 covariance matrix

    %% Step 2: QR Decomposition (RRQR)
    % Perform QR decomposition on Ryy. (For RRQR, one might use pivoting,
    % but here we assume Ryy's structure is such that the first num_sources rows
    % capture the signal subspace.)
    [~, R] = qr(Ryy);
    
    %% Step 3: Partition R and Form the Nullspace Basis
    % Partition R into R11 (num_sources x num_sources) and R12 (num_sources x (M-num_sources))
    R11 = R(1:num_sources, 1:num_sources);
    R12 = R(1:num_sources, num_sources+1:end);
    
    % Form the nullspace basis Psi:
    % Psi is constructed as:
    %   Psi = [ -inv(R11)*R12; I_{(M-num_sources)} ]
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
        % Steering vector for the full ULA:
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
    % For multiple sources, one could use findpeaks to extract the largest num_sources peaks.
    if num_sources == 1
        [~, peakIndex] = max(P);
        az_DOA_deg = xaxis(peakIndex);
    else
        [pks, locs] = findpeaks(P, 'SortStr','descend');
        num_found = min(num_sources, length(locs));
        az_DOA_deg = sort(xaxis(locs(1:num_found)));
    end
end
