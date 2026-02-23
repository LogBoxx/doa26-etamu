function a = create_uca_steering_vector_v2(theta_deg, phi_deg, k, ant_pos)
% create_uca_steering_vector
%
% This function computes the steering vector for a Uniform Circular Array (UCA).
%
% Inputs:
%   theta_deg   - Azimuth angle of the source (degrees)
%   phi_deg     - Elevation angle of the source (degrees)
%   k           - Wavenumber (2*pi/lambda)
%   ant_pos     - M-by-3 matrix of antenna element positions [x; y; z]
%   ant_type    - String specifying the antenna type: 'isotropic', 'dipole', or 'directional'
%   q           - Parameter for the directional 'cos^q' model (used only if ant_type is 'directional')
%
% Output:
%   a           - M-by-1 steering vector

% Convert angles to radians for calculations
theta = deg2rad(theta_deg);
phi = deg2rad(phi_deg);

M = size(ant_pos, 2); % Number of antennas
a = zeros(M, 1);      % Initialize steering vector

% Source direction unit vector
u = [sin(phi)*cos(theta); sin(phi)*sin(theta); cos(phi)];

% Loop through each antenna element
for m = 1:M
    % 1. Calculate Phase Shift (due to position)
    phase_shift = exp(1j * k * u' * ant_pos(:, m));

    % 2. Calculate Antenna Gain (due to element pattern)
    gain = 1; % Default for isotropic

    % Combine gain and phase for the element
    a(m) = gain * phase_shift;
end

end
