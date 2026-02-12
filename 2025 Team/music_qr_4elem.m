function [doa_deg, sp_dB, xaxis] = music_qr_4elem(rx_m, num_sources, dtheta_deg)
% music_qr_4elem performs DOA estimation using a MUSIC-like algorithm
% that employs QR decomposition instead of eigendecomposition.
% This version is optimized for 4-element arrays and scenarios with low numbers of snapshots.
%
% Inputs:
%   rx_m        - Received data matrix (4 x N snapshots) from a 4-element ULA.
%   num_sources - Number of signal sources to estimate.
%   dtheta_deg  - Angular resolution (in degrees) for the spectral search.
%
% Outputs:
%   doa_deg     - Estimated DOA(s) in degrees (vector of length num_sources).
%   sp_dB       - Computed spatial spectrum (in dB) over the angular grid.
%   xaxis       - Angular grid (in degrees) over which the spectrum is computed.
%
% The method:
%   1. Splits the 4-element array into two 2-element subarrays.
%   2. Computes the cross-correlation matrix between subarrays.
%   3. Applies QR decomposition on (Rxy*Rxy') and extracts the noise subspace.
%   4. Scans over a grid of candidate angles, computing the MUSIC spectrum.
%   5. Finds the peaks in the spectrum corresponding to the DOA(s).

    % Determine the total number of antennas and snapshots
    [total_antennas, N] = size(rx_m);
    
    % Validate inputs
    if total_antennas ~= 4
        error('This function is designed for 4-element arrays only.');
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
    
    M = total_antennas / 2;  % Number of antennas per subarray (2 for a 4-element array)
    
    % Define element spacing (in wavelengths) for the array
    d = 0.5;  % Half-wavelength spacing (typical for ULA)
    
    % Split the received data into two subarrays
    subarrayY = rx_m(1:M, :);      % First half (elements 1-2)
    subarrayX = rx_m(M+1:end, :);  % Second half (elements 3-4)
    
    % Compute the cross-correlation matrix between subarrays
    Rxy = (subarrayX * subarrayY') / N;  % 2x2 matrix
    
    % Form the product and perform QR decomposition
    [Q, ~] = qr(Rxy * Rxy');  % Q is 2x2 orthogonal matrix
    
    % Extract the noise subspace from Q:
    % For 2 antennas per subarray and K sources, columns K+1 to 2 span the noise subspace
    Q1 = Q(:, num_sources+1:end);
    
    % Form a projection matrix for the noise subspace
    En = Q1 * Q1';  % 2x2 matrix
    
    % Define the angular search grid
    xaxis = -90:dtheta_deg:90;  
    num_angles = length(xaxis);
    
    % Pre-allocate spectrum array
    SP = zeros(num_angles, 1);
    derad = pi/180;
    
    % Generate array element positions for the first subarray
    element_positions = (-M:-1).';  % (-2:-1)' for 4-element array, first subarray
    
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
