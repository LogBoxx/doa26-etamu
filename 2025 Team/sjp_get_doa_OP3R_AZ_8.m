function [az_DOA_deg, sp_board1_db_v, sp_board2_db_v] = sjp_get_doa_OP3R_AZ_8(rx_m, num_sources, dtheta_deg)
% SJP_GET_DOA_OP3R_AZ_8 Direction of Arrival estimation using an ESPRIT-like method
%
% This function performs azimuth-only DOA estimation using an ESPRIT-like method
% on data acquired from an 8-element ULA (from two FMComms5 boards).
%
% Inputs:
%   rx_m         - Received data matrix (8 x N snapshots), where:
%                  Rows 1-4: board1 (negative side)
%                  Rows 5-8: board2 (positive side)
%   num_sources  - Number of signal sources to estimate
%   dtheta_deg   - Angular resolution (in degrees) for the beamforming spectrum
%
% Outputs:
%   az_DOA_deg      - Estimated azimuth DOA (in degrees, sorted)
%   sp_board1_db_v  - Beamforming spectrum (dB) computed from board1 data
%   sp_board2_db_v  - Beamforming spectrum (dB) computed from board2 data
%
% Algorithm steps:
%   1. Splitting the full array into two subarrays
%   2. Computing a cross-correlation matrix
%   3. Forming an autocorrelation matrix
%   4. Using QR decomposition to extract the signal subspace
%   5. Forming a phase-shift matrix and estimating DOA from its eigenvalues

    %% Parameters
    M = 4;         % Number of elements per subarray (total of 8 elements)
    d = 0.5;      % Element spacing in wavelengths 
    % Determine the number of snapshots
    [~, N] = size(rx_m);
    
    % Define full array indices for an 8-element ULA: -4, -3, -2, -1, 0, 1, 2, 3
    arrayIndices = (-M:M-1).';  % 8x1 vector
    
    % Split the data into two subarrays:
    % Subarray1 from board1 (rows 1:4) corresponds to the "negative" side.
    subarray1 = rx_m(1:M, :);
    % Subarray2 from board2 (rows 5:8) corresponds to the "positive" side.
    subarray2 = rx_m(M+1:end, :);
    
    %% ESPRIT-like DOA Estimation
    % Compute cross-correlation between subarray2 and subarray1
    Rxy = (subarray2 * subarray1') / N;  
    
    % Form an autocorrelation-like matrix by stacking Rxy and its conjugate transpose
    Rz = [Rxy; Rxy'];
    
    % Perform QR decomposition on Rz
    [~, Rn] = qr(Rz);
    
    % Extract the signal subspace:
    % Select the first num_sources rows of Rn and then transpose to get a matrix 
    signalSubspace = Rn(1:num_sources, :)';
    
    % Form the phase-shift matrix Psi using consecutive rows of the signal subspace
    Psi = pinv(signalSubspace(1:end-1, :)) * signalSubspace(2:end, :);
    
    % Compute the eigenvalues of Psi
    eig_vals = eig(Psi);
    
    % Estimate the azimuth DOA from the phase angles
    az_DOA_deg = sort( asin( angle(eig_vals) / (2*pi*d) ) * (180/pi) );
    
    %% Beamforming Spectrum Calculation for Visualization
    % Create an azimuth scan grid based on the given resolution
    xaxis = -90:dtheta_deg:90;
    num_angles = length(xaxis);
    
    % Split the array indices for the two boards:
    indices_board1 = arrayIndices(1:M);   % Board1: [-4; -3; -2; -1]
    indices_board2 = arrayIndices(M+1:end); % Board2: [0; 1; 2; 3]
    
    % Compute sample covariance matrices for each subarray
    R1 = (subarray1 * subarray1') / N;
    R2 = (subarray2 * subarray2') / N;
    
    % Initialize spectrum vectors
    sp_board1 = zeros(1, num_angles);
    sp_board2 = zeros(1, num_angles);
    
    % Compute the beamforming response for each angle in the scan grid
    for idx = 1:num_angles
        theta_deg = xaxis(idx);
        theta_rad = theta_deg * pi / 180;
        
        % Steering vectors for board1 and board2
        a1 = exp(1j * 2*pi*d * indices_board1 * sin(theta_rad));
        a2 = exp(1j * 2*pi*d * indices_board2 * sin(theta_rad));
        
        % Beamformer response (quadratic form using the covariance matrices)
        sp_board1(idx) = abs(a1' * R1 * a1);
        sp_board2(idx) = abs(a2' * R2 * a2);
    end
    
    % Normalize and convert the spectra to decibel (dB) scale
    sp_board1_db_v = 10 * log10( sp_board1 / max(sp_board1) );
    sp_board2_db_v = 10 * log10( sp_board2 / max(sp_board2) );
end
