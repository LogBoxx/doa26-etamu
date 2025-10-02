function a = create_uca_steering_vector(theta_deg, phi_deg, k, ant_pos, ant_type, q)
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
    switch lower(ant_type)
        case 'isotropic'
            gain = 1;

        case 'dipole'
            % Gain pattern for a z-oriented dipole is sin(phi)
            gain = abs(sin(phi)); % abs() to ensure non-negative gain

        case 'directional'
            % Gain pattern is cos^q(psi_m), where psi_m is the angle
            % between the source direction and the antenna's main beam.
            % For a UCA in the xy-plane, antennas are pointed radially outward.
            ant_angle_m = atan2(ant_pos(2, m), ant_pos(1, m));
            
            % Main beam direction vector for antenna m
            u_m = [cos(ant_angle_m); sin(ant_angle_m); 0];

            % Angle psi_m
            cos_psi_m = u' * u_m;

            % The paper uses the pattern for field, not power, in the steering
            % vector. We only consider positive values for the gain pattern.
            if cos_psi_m > 0
                gain = cos_psi_m ^ q;
            else
                gain = 0; % Antenna has no gain in the back-hemisphere
            end
    end

    % Combine gain and phase for the element
    a(m) = gain * phase_shift;
end

end
