function [doa_deg, sp_dB, xaxis] = sjp_get_doa_MUSIC_QR(rx_m, num_sources, dtheta_deg)
% SJP_GET_DOA_MUSIC_QR Direction of Arrival estimation using MUSIC with QR decomposition.
%
% Purpose:
%   Performs Direction of Arrival (DOA) estimation for signals impinging on a
%   Uniform Linear Array (ULA) using a variant of the MUltiple SIgnal Classification
%   (MUSIC) algorithm. This implementation utilizes QR decomposition instead of
%   the standard eigenvalue decomposition (EVD) of the covariance matrix. This
%   approach can be particularly effective in scenarios with a low number of
%   available snapshots (e.g., 10-15).
%
% Array Assumption:
%   - Assumes an even number of total antennas (e.g., 8).
%   - Assumes the input data `rx_m` is ordered such that the first half of the
%     rows correspond to the elements on one side of the array center (e.g., negative
%     indices [-M .. -1]) and the second half correspond to the elements on the
%     other side (e.g., positive indices [0 .. M-1]). M = total_antennas / 2.
%   - Assumes half-wavelength spacing (d=0.5) between elements.
%
% Algorithm Steps:
%   1. Splits the input data matrix `rx_m` into two subarrays, `subarrayY` (first M rows)
%      and `subarrayX` (last M rows).
%   2. Computes the cross-correlation matrix `Rxy = (subarrayX * subarrayY') / N`, where N
%      is the number of snapshots.
%   3. Forms the product `Rxy * Rxy'` (an M x M matrix).
%   4. Performs QR decomposition on this product: `[Q, ~] = qr(Rxy * Rxy')`.
%   5. Extracts the noise subspace estimate from the orthogonal matrix `Q`. The columns
%      of `Q` corresponding to the smaller singular values (or equivalently, the last
%      M - num_sources columns if sorted) span the noise subspace. Here, columns
%      `num_sources + 1` to `M` are taken as the noise subspace basis `Q1`.
%   6. Forms the projection matrix onto the noise subspace: `En = Q1 * Q1'`.
%   7. Defines an angular search grid `xaxis` from -90 to +90 degrees with resolution `dtheta_deg`.
%   8. Defines the steering vector `a` corresponding to the *first* subarray (`subarrayY`,
%      elements -M to -1).
%   9. Scans through each angle `theta` in the grid:
%      a. Computes the steering vector `a(theta)` for that angle.
%      b. Calculates the MUSIC pseudo-spectrum value: `P(theta) = 1 / (a(theta)' * En * a(theta))`.
%         The denominator approaches zero when `a(theta)` is orthogonal to the noise subspace
%         (i.e., lies in the signal subspace), resulting in peaks at the DOAs.
%  10. Normalizes the spectrum `P` and converts it to decibels (`sp_dB`).
%  11. Finds the locations of the peaks in the spectrum `P` (or `sp_dB`) to determine the
%      estimated DOAs (`doa_deg`). Handles finding single or multiple sources.
%
% Inputs:
%   rx_m        - Complex double matrix (total_antennas x N) containing the snapshot data
%                 from the ULA, ordered as described above. N is the number of snapshots.
%   num_sources - Integer, the expected number of signal sources.
%   dtheta_deg  - Double, the angular resolution in degrees for the spectral search grid.
%
% Outputs:
%   doa_deg     - Double row vector containing the estimated DOA(s) in degrees, sorted
%                 ascendingly. Length is <= `num_sources`.
%   sp_dB       - Double column vector containing the computed MUSIC spatial spectrum
%                 in decibels (dB), normalized to a maximum of 0 dB.
%   xaxis       - Double row vector, the angular grid (in degrees) corresponding to `sp_dB`.
%
% Warnings/Errors:
%   - Error if `total_antennas` is not even.
%   - Warning if the number of snapshots `N` is very low (< 5) or high (> 50), as the
%     algorithm is tuned for low snapshot counts.
%   - Error if `N` is less than `num_sources + 1`, which is required for subspace estimation.
%   - Warning if fewer peaks are found than `num_sources` requested.

    % Determine the total number of antennas and snapshots
    [total_antennas, N] = size(rx_m);
    
    % Validate inputs
    if mod(total_antennas, 2) ~= 0
        error('Number of antennas must be even to split into two subarrays.');
    end
    
    % Check if the number of snapshots is appropriate
    if N < 5
        warning('Very few snapshots (%d) detected. Results may be unreliable.', N);
    elseif N > 50
        warning('Large number of snapshots (%d) detected. This algorithm is optimized for 10-15 snapshots.', N);
    end
    
    % Ensure we have at least the minimum required snapshots
    min_required = num_sources + 1;
    if N < min_required
        error('Number of snapshots (%d) must be at least num_sources+1 (%d).', N, min_required);
    end
    
    M = total_antennas / 2;  % Number of antennas per subarray
    
    % Define element spacing (in wavelengths) for the array
    d = 0.5;  % Half-wavelength spacing
    
    % Split the received data into two subarrays
    subarrayY = rx_m(1:M, :);      % First half (negative side in this configuration)
    subarrayX = rx_m(M+1:end, :);  % Second half (positive side in this configuration)
    
    % Compute the cross-correlation matrix between subarrays
    Rxy = (subarrayX * subarrayY') / N;  % M x M matrix
    
    % Form the product and perform QR decomposition
    [Q, ~] = qr(Rxy * Rxy');  % Q is MxM orthogonal matrix
    
    % Extract the noise subspace from Q:
    % For M antennas and K sources, columns K+1 to M span the noise subspace
    Q1 = Q(:, num_sources+1:end);
    
    % Form a projection matrix for the noise subspace
    En = Q1 * Q1';  % M x M matrix
    
    % Define the angular search grid
    xaxis = -90:dtheta_deg:90;  
    num_angles = length(xaxis);
    
    % Pre-allocate spectrum array
    SP = zeros(num_angles, 1);
    derad = pi/180;
    
    % Generate array element positions for the first subarray
    element_positions = (-M:-1).';  % Mx1 vector for negative side elements
    
    % Scan over candidate angles and compute the spatial spectrum
    for idx = 1:num_angles
        theta_deg = xaxis(idx);
        theta_rad = theta_deg * derad;
        
        % Steering vector for the given angle (M x 1)
        a = exp(-1j * 2 * pi * d * element_positions * sin(theta_rad));
        
        % Compute the MUSIC spatial spectrum value at this angle
        % The denominator approaches zero when a is in the signal subspace
        denominator = a' * En * a;
        if abs(denominator) < 1e-10  % Prevent division by very small numbers
            SP(idx) = 1e10;  % Large number to represent peaks
        else
            SP(idx) = 1 / abs(denominator);
        end
    end
    
    % Normalize and convert the spectrum to dB scale
    SP = abs(SP);
    SPmax = max(SP);
    sp_dB = 10 * log10(SP / SPmax);
    
    % Find peaks in the spatial spectrum
    if num_sources == 1
        [~, peakIndex] = max(SP);
        doa_deg = xaxis(peakIndex);
    else
        % For multiple sources, use findpeaks with sorting
        [~, locs] = findpeaks(sp_dB, 'SortStr', 'descend');
        
        if isempty(locs)
            % If no peaks found, use the highest points instead
            [~, sorted_indices] = sort(sp_dB, 'descend');
            locs = sorted_indices(1:min(num_sources, length(sorted_indices)));
        elseif length(locs) < num_sources
            % If fewer peaks than requested sources, take what we found
            warning('Found only %d peaks when %d sources were requested.', ...
                    length(locs), num_sources);
        end
        
        % Take at most num_sources peaks
        locs = locs(1:min(num_sources, length(locs)));
        doa_deg = sort(xaxis(locs));
    end
end