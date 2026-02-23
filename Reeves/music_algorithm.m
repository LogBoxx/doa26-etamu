function theta_est = music_algorithm(X, D, k, ant_pos, theta_scan, phi_scan, ant_type, q)
% music_algorithm
%
% Implements the MUSIC (Multiple Signal Classification) algorithm for DOA estimation.
%
% Inputs:
%   X           - M-by-N matrix of received signal snapshots
%   D           - Number of sources
%   k           - Wavenumber (2*pi/lambda)
%   ant_pos     - M-by-3 matrix of antenna element positions [x; y; z]
%   theta_scan  - Vector of azimuth angles to search (degrees)
%   phi_scan    - Scalar elevation angle (degrees)
%   ant_type    - String for antenna type ('isotropic', 'dipole', 'directional')
%   q           - Parameter for directional model
%
% Output:
%   theta_est   - Estimated azimuth angle in degrees

[M, ~] = size(X);

% 1. Calculate the sample covariance matrix
Rxx = (X * X') / size(X, 2);

% 2. Perform eigenvalue decomposition
[E, V] = eig(Rxx);
eigenvalues = diag(V);

% Sort eigenvalues and eigenvectors
[~, idx] = sort(eigenvalues, 'descend');
E = E(:, idx);

% 3. Identify the noise subspace
% The noise subspace is spanned by eigenvectors corresponding to the
% M-D smallest eigenvalues.
En = E(:, D+1:end);

% 4. Calculate MUSIC spectrum
music_spectrum = zeros(size(theta_scan));
for i = 1:length(theta_scan)
    % Create a steering vector for the current scan angle
    a_scan = create_uca_steering_vector(theta_scan(i), phi_scan, k, ant_pos, ant_type, q);

    % Project onto the noise subspace
    denominator = a_scan' * (En * En') * a_scan;

    music_spectrum(i) = 1 / abs(denominator);
end

% 5. Find the peak in the spectrum
[~, peak_idx] = max(music_spectrum);
theta_est = theta_scan(peak_idx);

end
