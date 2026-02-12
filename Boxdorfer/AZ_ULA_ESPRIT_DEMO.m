%% AZ_ULA_ESPRIT_DEMO.m
%% Logan's work (God tier)

clear; clc; close all;

snr_db = -6;
Navg = 10;

source_angles = 0;

algorithm = 'ESPRIT_REF';

num_sources = 1;
dtheta_deg = 0.25;
numtrials = 10000;
num_snapshots = 2^9;
CenterFrequency = 2.4e9;
SamplingRate = 30.72e6;
SamplesPerFrame = 2^14;

max_reconnect = 2;  % Maximum number of retry attempts for data acquisition
max_errors = 5;     % Maximum number of consecutive errors allowed
num_error = 0;

try
    % Example for a 4-channel FMComms5 setup (adjust according to your hardware)
    rx_device = adi.FMComms5.Rx('uri','ip:192.168.0.101', ...
        'CenterFrequency', CenterFrequency, ...
        'SamplingRate', SamplingRate, 'SamplesPerFrame', SamplesPerFrame);
    rx_device.EnabledChannels = [1 2 3 4];  % Enable all 4 channels
catch ME
    warning('Hardware setup failed. Running in simulation mode.');
    fprintf('Error: %s\n', ME.message);
    simulation_mode = true;
end

if ~exist('simulation_mode', 'var')
    simulation_mode = false;
end

if ~exist('figNum', 'var')
    figNum = 1;
end

az_DOA_deg_m = zeros(numtrials, num_sources);
xaxis_sp_v = linspace(-90, 90, 180/dtheta_deg + 1);

az_hist = nan(Navg, num_sources);

for t = 1:numtrials
    try
        % Acquire data (either from hardware or simulation)
        if simulation_mode
            % Generate simulated signal for testing
            % Example: Single source at 30 degrees
            rx_m = generate_simulated_data(4, 1000, source_angles, snr_db);
        else
            % Acquire data from hardware using get_data_fmc5_4elem function
            rx_m = get_data_fmc5_4elem(rx_device, max_reconnect);
        end
        
        % Reduce the raw samples to the desired number of snapshots
        rx_snapshots = reduce_snapshots(rx_m, num_snapshots);
        
        % Call DOA estimation algorithm (reference ESPRIT + full-array Bartlett spectrum)
        [az_DOA_deg, sp_dB, xaxis] = esprit_ref_4elem_with_spectrum(rx_snapshots, num_sources, dtheta_deg);

        az_hist = [az_hist(2:end,:); az_DOA_deg(:).'];
        az_avg = mean(az_hist, 1, 'omitnan');

        update_live_plot(algorithm, t, az_DOA_deg, az_avg, xaxis, sp_dB, num_snapshots);

        % Store the estimated azimuth DOA for final statistics
        az_DOA_deg_m(t, :) = az_avg;
        
        % Print current estimate to console
        fprintf('Trial %d: Estimated DOA = %0.2f deg | Avg(10) = %0.2f deg\n', ...
            t, az_DOA_deg, az_avg);
        
    catch ME
        disp("Error in trial " + t + ": " + ME.message);
        num_error = num_error + 1;
        t = t - 1;  % Retry the current trial
        if num_error >= max_errors
            disp("Exceeded maximum error count. Aborting.");
            rethrow(ME);
        end
    end
end

if ~simulation_mode
    rx_device.release();
end

figure(figNum);
est_std = std(az_DOA_deg_m(:));
scatter(az_DOA_deg_m, zeros(size(az_DOA_deg_m)), 'x', 'LineWidth', 1);
title(["Azimuth-Only DOA (4-Element ULA)", "Std. = " + num2str(est_std), ...
       "Mean = " + num2str(mean(az_DOA_deg_m(:))) + " deg"]);
xlabel("Azimuth (deg)");
ylabel("Elevation (not used)");
xlim([-90, 90]);
%ylim([-10, 10]);
grid on;
figNum = figNum + 1;

%% PLOT FUNCTION
function update_live_plot(algorithm, t, az_DOA_deg, az_avg, varargin)
    % Get the number of snapshots (last argument)
    num_snapshots_arg = varargin{end};

    
    figure(100); clf;  % Use a consistent figure number for live plotting
    
    % Plot spectrum based on the algorithm
    if false % (kept for legacy ESPRIT plotting)
        xaxis_sp_v = varargin{1};
        sp_board1_db_v = varargin{2};
        sp_board2_db_v = varargin{3};
        % num_snapshots_arg is varargin{4}
        
        % Combined spectrum plot
        subplot(2,1,1);
        plot(xaxis, sp_dB, 'm', 'LineWidth', 2); hold on;
        xline(az_avg, '--k', 'LineWidth', 2);
        hold off;
        xlabel("Azimuth (deg)");
        ylabel("Spatial Spectrum (dB)");
        title("Full-Array Spatial Spectrum");
        xlim([-90, 90]); ylim([-50, 0]);
        grid on;

    else % ESPRIT_REF (full-array spectrum)
        xaxis = varargin{1};
        sp_dB = varargin{2};
        % num_snapshots_arg is varargin{3}
        
        % Full spatial spectrum plot
        subplot(2,1,1);
        plot(xaxis, sp_dB, 'm', 'LineWidth', 2);
        xlabel("Azimuth (deg)");
        ylabel("Spatial Spectrum (dB)");
        title("Full-Array Spatial Spectrum");
        xlim([-90, 90]); ylim([-50, 0]);
        xline(az_avg);
        grid on;
    end
    
    % DOA estimate plot - common for all algorithms
    subplot(2,1,2);
    %scatter(az_DOA_deg, zeros(size(az_DOA_deg)), 300, 'x', 'LineWidth', 4);
    scatter(az_avg, zeros(size(az_DOA_deg)), 300, 'x', 'LineWidth', 4);
    title("DOA Estimate, Trial: " + num2str(t));
    xlabel("Azimuth (deg)");
    xlim([-90, 90]); ylim([-10, 10]); grid on;
    
    % Use the explicitly passed number of snapshots
    sgtitle("Azimuth-Only DOA (" + algorithm + ") - Using " + num2str(num_snapshots_arg) + " snapshots");
    drawnow;  % Ensure immediate plot update
end

%% FUNCTIONS
function [az_DOA_deg, sp_dB, xaxis] = esprit_ref_4elem_with_spectrum(rx_m, num_sources, dtheta_deg)

    [M,N] = size(rx_m);
    if M ~= 4
        error('Expected rx_m as 4xN for 4-element ULA.');
    end

    d = 0.5; % element spacing in wavelengths (matches your sim + typical half-lambda ULA)

    % --- ESPRIT DOA W FORWARD BACKWARD AVERAGING ---
    
    R = (rx_m*rx_m')/N;

    % --- Forward Backward Averaging ---
    J = flipud(eye(M));
    R = 0.5 * (R + J*conj(R)*J);
    % ----------------------------------------

    [U,D] = eig(R);
    [~,idx] = sort(diag(D),'descend');
    U = U(:,idx);

    Es = U(:,1:num_sources);  % 4xK

    % Overlapping subarrays: [1 2 3] and [2 3 4]
    % DONT CHANGE J1 J2 E1 E2
    J1 = [eye(3), zeros(3,1)];
    J2 = [zeros(3,1), eye(3)];

    E1 = J1*Es;
    E2 = J2*Es;

    Psi = pinv(E1)*E2;
    ev = eig(Psi);

    u = angle(ev)/(2*pi*d);          % u = sin(theta)
    u = max(min(real(u),1),-1);      % clamp numerical spill
    az_DOA_deg = sort(asind(u));

    % --- Full-array pseudo spectrum for plotting ---
    xaxis = linspace(-90, 90, 180/dtheta_deg + 1);
    % Centered positions (origin doesn't matter for Bartlett magnitude)
    pos = (-(M-1)/2 : (M-1)/2).';

    sp = zeros(size(xaxis));
    for i = 1:numel(xaxis)
        th = xaxis(i)*pi/180;
        a = exp(1j*2*pi*d*pos*sin(th));
        sp(i) = real(a' * R * a);
    end
    sp = sp ./ max(sp + eps);
    sp_dB = 10*log10(sp);
end

function rx_m = generate_simulated_data(num_antennas, num_samples, source_angles, snr_db)

    d = 0.5;  % Half-wavelength spacing
    num_sources = length(source_angles);
    
    % Array geometry (centered around origin)
    array_positions = (-(num_antennas-1)/2 : (num_antennas-1)/2)';
    
    % Initialize received signal matrix
    rx_m = zeros(num_antennas, num_samples);
    
    % Generate source signals (complex Gaussian)
    source_signals = (randn(num_sources, num_samples) + 1j*randn(num_sources, num_samples))/sqrt(2);
    
    % Compute steering matrix
    A = zeros(num_antennas, num_sources);
    for i = 1:num_sources
        theta_rad = source_angles(i) * pi/180;
        A(:,i) = exp(1j * 2*pi*d * array_positions * sin(theta_rad));
    end
    
    % Generate noiseless received signal
    signal_component = A * source_signals;
    

    % NOISE
    % Add noise based on SNR
    signal_power = mean(abs(signal_component(:)).^2);
    noise_power = signal_power / (10^(snr_db/10));
    noise = sqrt(noise_power/2) * (randn(num_antennas, num_samples) + 1j*randn(num_antennas, num_samples));
    
    % Final received signal
    rx_m = signal_component + noise;
end

% Just keep this
function [rx_m] = get_data_fmc5_4elem(rx_device, NUM_RETRY)

    valid = false;
    j = 0; % Tracks # of times data has been retrieved (but has not been valid)
    k = 0; % Tracks # of connection attempts
    
    while ~valid
        try
            % Acquire data from the device
            raw_data = rx_device();
            rx_m = fliplr(raw_data);

            rx_m = raw_data';  % Transpose to get channels in rows
            
            % Basic validity check (non-zero data, no NaNs, proper size)
            valid = all(size(rx_m) > 0) && ~any(isnan(rx_m(:))) && sum(abs(rx_m(:))) > 0;
        catch ME
            fprintf("Retrying connection...\n");
            pause(1);
            k = k + 1;
            if k >= NUM_RETRY
                fprintf("Failed to connect to device\n");
                rethrow(ME);
            end
        end
        
        if exist('rx_m', 'var') && ~valid
            fprintf("Invalid data...\n");
            j = j + 1;
        end
        
        if (j >= NUM_RETRY) && ~valid
            error("Connecting but not grabbing valid data for some reason");
        end
    end
end
