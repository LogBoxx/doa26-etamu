%% DOA_Smoothed_Tracking_Hardware_Demo.m
% Main script for real-time, azimuth-only Direction of Arrival (DOA) estimation
% using two ADI FMComms5 boards configured as an 8-element Uniform Linear Array (ULA).
% This script performs DOA estimation on hardware data, applies exponential
% smoothing (alpha=0.4) to stabilize the estimates, and displays the results live.
%
% Hardware Configuration Assumption:
%   - Two FMComms5 boards are used.
%   - Board 1 (e.g., IP 192.168.1.1) provides data for the first 4 ULA elements.
%   - Board 2 (e.g., IP 192.168.0.1) provides data for the last 4 ULA elements.
%   - The get_data_fmc5_OptB function handles data retrieval and combines data such
%     that the resulting 8xN matrix corresponds to elements ordered spatially
%     (e.g., from element -4 to +3 relative to the array center).
%
% Workflow:
%   1. Initializes and configures the two FMComms5 receiver objects.
%   2. Enters a loop running for a specified number of trials (`numtrials`).
%   3. In each trial:
%      a. Acquires synchronized data frames from both boards using `get_data_fmc5_OptB`.
%      b. Reduces the number of samples per frame down to a smaller number of
%         'snapshots' using averaging (`reduce_snapshots`). This reduces computational
%         load and can improve covariance matrix estimation.
%      c. Performs DOA estimation on the snapshot matrix using the selected algorithm
%         ('MUSIC_QR', 'RQR', or 'OP3R_AZ'). These are high-resolution subspace methods.
%      d. Applies exponential smoothing to the current raw DOA estimate(s) using the
%         previous smoothed estimate to reduce jitter and improve tracking stability.
%      e. Updates live plots showing the spatial spectrum (output of the DOA algorithm)
%         and the history of the smoothed DOA estimate(s) over the trials.
%   4. Releases hardware resources after the loop finishes.


clear; clc; close all;

%% User Options
% --- Algorithm & Parameters ---
% Choose the DOA estimation algorithm:
%   'MUSIC_QR': MUSIC variant using QR decomposition, suitable for low snapshots.
%   'RQR'     : Rank-Revealing QR based search algorithm.
%   'OP3R_AZ' : ESPRIT-like algorithm adapted for the 8-element, two-board setup.
algorithm = 'MUSIC_QR'; % CHANGE THIS TO SELECT ALGORITHM

num_sources = 1;         % Expected number of signal sources impacting the array.
dtheta_deg = 0.05;       % Angular resolution (step size) in degrees for the spectral search
                         % performed by the DOA algorithms. Finer resolution yields smoother
                         % spectrum plots but increases computation time.
                         
numtrials = 100;         % Total number of data acquisition and estimation cycles to run.
num_snapshots = 15;      % Target number of snapshots after sample reduction. Subspace methods
                         % often perform well with a relatively small number of snapshots
                         % (e.g., 10-20), especially MUSIC_QR.

% --- Smoothing Parameter ---
alpha = 0.4;  % Exponential smoothing factor (0 <= alpha <= 1).
              % Controls the trade-off between responsiveness and smoothness.
              % alpha=0 means the output is always the previous estimate (no update).
              % alpha=1 means the output is always the current raw estimate (no smoothing).
              % Values between 0 and 1 blend the new estimate with the previous one.

% --- RF Configuration ---
% These parameters must match the hardware configuration.
CenterFrequency = 2.4e9; % Center frequency of reception in Hz (e.g., 2.4 GHz).
SamplingRate = 30e6;     % Sampling rate in Hz (e.g., 30 MHz). Must be consistent across boards.
SamplesPerFrame = 2^15;  % Number of raw samples collected per channel in each acquisition
                         % from the hardware buffer (e.g., 32768). Affects processing time
                         % and the input size to reduce_snapshots.

% --- Error Handling ---
max_reconnect = 2;       % Maximum number of consecutive attempts to reconnect or retry
                         % data acquisition if an error occurs.
max_errors = 5;          % Maximum number of consecutive errors allowed before aborting the script.
num_error = 0;           % Counter for consecutive errors.

%% Hardware Setup
rx_board1 = []; % Initialize hardware object handles
rx_board2 = [];

% FMComms5 Receiver Setup for Two Boards
try
    disp('Setting up hardware...');
    % Board 1: Corresponds to the first 4 elements of the ULA.
    % URI specifies the IP address of the first board.
    rx_board1 = adi.FMComms5.Rx('uri','ip:192.168.1.1', ... % IP Address for Board 1
        'CenterFrequency', CenterFrequency, 'CenterFrequencyChipB', CenterFrequency, ...
        'SamplingRate', SamplingRate, 'SamplesPerFrame', SamplesPerFrame);
    rx_board1.EnabledChannels = [1 2 3 4]; % Enable the 4 Rx channels on this board.

    % Board 2: Corresponds to the last 4 elements of the ULA.
    % URI specifies the IP address of the second board.
    rx_board2 = adi.FMComms5.Rx('uri','ip:192.168.0.1', ... % IP Address for Board 2
        'CenterFrequency', CenterFrequency, 'CenterFrequencyChipB', CenterFrequency, ...
        'SamplingRate', SamplingRate, 'SamplesPerFrame', SamplesPerFrame);
    rx_board2.EnabledChannels = [1 2 3 4]; % Enable the 4 Rx channels on this board.
    disp('Hardware setup complete.');
catch ME
    % Catch errors during hardware initialization (e.g., incorrect IP, device not found).
    warning(ME.identifier, 'Hardware setup failed: %s\nCannot proceed without hardware.', ME.message);
    % Attempt to release any partially opened devices before exiting.
    if ~isempty(rx_board1) && isvalid(rx_board1)
        rx_board1.release();
    end
     if ~isempty(rx_board2) && isvalid(rx_board2)
        rx_board2.release();
    end
    error('Hardware initialization failed. Exiting script.'); % Stop script execution.
end

% --- Figure Numbering ---
% Ensures consistent figure handles if this script is called multiple times.
if ~exist('figNum','var')
    figNum = 1;
end

%% Preallocate Arrays for DOA Estimates
% Preallocate a matrix to store the smoothed DOA estimates for each trial.
% Rows correspond to trials, columns correspond to sources (if num_sources > 1).
% Initialized with NaN (Not a Number).
smoothed_DOA_deg_m = NaN(numtrials, num_sources);
% Create a vector representing the trial numbers for plotting on the x-axis.
trial_numbers = (1:numtrials)';

%% Set up the real-time display
% Create a figure window for live plotting.
live_fig = figure('Position', [100, 100, 900, 700], 'Name', ['Live DOA Estimation (Hardware) - Algorithm: ', algorithm]);

% Subplot 1: Spatial Spectrum Display
subplot(2, 1, 1);
% Plot the angular spectrum calculated by the DOA algorithm.
% Peaks in the spectrum indicate estimated Directions of Arrival.
h_spectrum1 = plot(NaN, NaN, 'm', 'LineWidth', 2); % Handle for primary spectrum (MUSIC/RQR) or Board 1 (OP3R_AZ)
hold on;
h_spectrum2 = plot(NaN, NaN, 'r', 'LineWidth', 1.5); % Handle for Board 2 spectrum (OP3R_AZ only)
set(h_spectrum2, 'Visible', 'off'); % Initially hide the second spectrum plot
xlabel('Azimuth (deg)');           % X-axis label
ylabel('Spatial Spectrum (dB)');   % Y-axis label (normalized power in decibels)
title_str_spec = sprintf('%s Spatial Spectrum', algorithm); % Initial title
title(title_str_spec);
xlim([-90, 90]);                   % Azimuth range from -90 to +90 degrees
ylim([-40, 5]);                    % Spectrum power range in dB (adjust as needed)
grid on;                           % Display grid lines
legend_spec = {'Spectrum'};        % Default legend entry
if strcmp(algorithm, 'OP3R_AZ')    % Specific setup for OP3R_AZ which outputs two spectra
    set(h_spectrum1, 'Color', 'b'); % Use blue for Board 1 spectrum
    legend_spec = {'Board 1 Spectrum', 'Board 2 Spectrum'}; % Update legend
    set(h_spectrum2, 'Visible', 'on'); % Make second plot visible
end
legend(legend_spec, 'Location', 'best'); % Add legend to the plot
hold off;

% Subplot 2: Smoothed DOA Tracking History Display
subplot(2, 1, 2);
% Plot the smoothed DOA estimate(s) over successive trials.
h_doa_lines = gobjects(num_sources, 1); % Array to hold plot handles for each source's line
colors = lines(num_sources); % Generate distinct colors for multiple source lines
for src = 1:num_sources % Create a line object for each expected source
    h_doa_lines(src) = plot(NaN, NaN, '-o', 'LineWidth', 1.5, ...
                           'MarkerSize', 4, 'Color', colors(src,:));
    hold on;
end
hold off;
xlabel('Trial Number');                  % X-axis label
ylabel('Smoothed DOA Estimate (degrees)'); % Y-axis label
title_str_track = sprintf('Smoothed DOA Tracking History (alpha=%.1f)', alpha); % Initial title
title(title_str_track);
grid on;                                 % Display grid lines
xlim([1, numtrials]);                    % X-axis range from 1 to total trials
ylim([-90, 90]);                         % Y-axis DOA range
% Create legend for tracking plot only if expecting more than one source
legend_track = {};
if num_sources > 1
    for src = 1:num_sources
        legend_track{end+1} = sprintf('Source %d', src); % Label each source line
    end
    legend(h_doa_lines, legend_track, 'Location', 'best'); % Add legend
end


%% Main Loop: Data Acquisition and DOA Estimation
% Loop through the specified number of trials.
for t = 1:numtrials
    try
        % --- Acquire Data ---
        % Check if hardware handles are still valid before attempting acquisition.
        if isempty(rx_board1) || ~isvalid(rx_board1) || isempty(rx_board2) || ~isvalid(rx_board2)
             error('Hardware connection lost or invalid.'); % Throw error if connection lost
        end
        % Call the function to get data from both boards. This function handles
        % polling the devices and combines the data into an 8xN matrix.
        % IMPORTANT: get_data_fmc5_OptB applies a flip to board 1 data internally
        % to ensure correct spatial ordering [-4 .. +3] for the ULA algorithms.
        raw_samples_m = get_data_fmc5_OptB(rx_board1, rx_board2, max_reconnect);

        % --- Reduce Snapshots ---
        % Reduce the number of samples (columns) to the desired number of snapshots
        % using the external reduce_snapshots.m function (via segment averaging).
        rx_snapshots = reduce_snapshots(raw_samples_m, num_snapshots);

        % --- DOA Estimation (Select Algorithm) ---
        % Initialize variables for DOA results.
        az_DOA_deg_vec = []; % Vector to store raw DOA estimates for this trial
        sp_dB1 = [];         % Primary spatial spectrum (or Board 1 for OP3R_AZ)
        sp_dB2 = [];         % Board 2 spatial spectrum (for OP3R_AZ)
        xaxis = [];          % Angular axis for the spectrum plot

        % Execute the chosen DOA algorithm.
        switch algorithm
            case 'MUSIC_QR'
                % Call the MUSIC-QR function for 8 elements.
                [az_DOA_deg_vec, sp_dB1, xaxis] = sjp_get_doa_MUSIC_QR(rx_snapshots, num_sources, dtheta_deg);
            case 'RQR'
                 % Call the RQR search function for 8 elements.
                [az_DOA_deg_vec, sp_dB1, xaxis] = sjp_get_doa_RQR_search_az(rx_snapshots, num_sources, dtheta_deg);
            case 'OP3R_AZ'
                 % Call the OP3R_AZ (ESPRIT-like) function for 8 elements.
                [az_DOA_deg_vec, sp_dB1, sp_dB2] = sjp_get_doa_OP3R_AZ_8(rx_snapshots, num_sources, dtheta_deg);
                % Reconstruct the angular axis if the algorithm doesn't return it.
                if ~isempty(sp_dB1)
                    xaxis = linspace(-90, 90, length(sp_dB1));
                else
                    xaxis = -90:dtheta_deg:90; % Default axis if spectrum is empty
                end
            otherwise
                error('Unknown algorithm selected: %s', algorithm); % Handle invalid selection
        end

        % --- Process Raw Results ---
        % Handle cases where the algorithm finds fewer sources than expected or no sources.
        if isempty(az_DOA_deg_vec)
            current_raw_doa = NaN(1, num_sources); % Assign NaN if no sources found
            warning('Trial %d: No sources found by %s.', t, algorithm);
        elseif length(az_DOA_deg_vec) < num_sources
             % If fewer sources found, pad the result vector with NaN.
             warning('Trial %d: Found %d sources with %s, expected %d. Padding with NaN.', t, length(az_DOA_deg_vec), algorithm, num_sources);
             current_raw_doa = NaN(1, num_sources);
             current_raw_doa(1:length(az_DOA_deg_vec)) = sort(az_DOA_deg_vec); % Store the found DOAs (sorted)
        else
            % If enough sources found, sort them and take the expected number.
            current_raw_doa = sort(az_DOA_deg_vec);
            current_raw_doa = current_raw_doa(1:num_sources); % Ensure correct size
        end
        % Ensure the result is a row vector for consistent processing.
        current_raw_doa = reshape(current_raw_doa, 1, num_sources);

        % --- Apply Smoothing (Alpha = 0.4) ---
        % Apply exponential smoothing to each source's estimate.
        current_smoothed_doa = NaN(1, num_sources); % Initialize smoothed value for this trial
        for src = 1:num_sources
            % Handle cases where the raw estimate for this source is NaN.
            if isnan(current_raw_doa(src))
                % If raw is NaN, propagate the previous smoothed value (or NaN if first trial).
                if t > 1
                    current_smoothed_doa(src) = smoothed_DOA_deg_m(t-1, src);
                else
                    current_smoothed_doa(src) = NaN;
                end
                continue; % Skip smoothing calculation for this source
            end

            % Apply smoothing formula: S[t] = alpha*Y[t] + (1-alpha)*S[t-1]
            % where S is smoothed value, Y is raw value, t is current trial.
            if t == 1 || isnan(smoothed_DOA_deg_m(t-1, src))
                 % Initialize smoothed value with the raw value for the first trial
                 % or if the previous smoothed value was NaN.
                 current_smoothed_doa(src) = current_raw_doa(src);
            else
                 % Calculate the smoothed value using the formula.
                 current_smoothed_doa(src) = alpha * current_raw_doa(src) + (1-alpha) * smoothed_DOA_deg_m(t-1, src);
            end
        end
        % Store the calculated smoothed DOA estimate(s) for this trial in the history matrix.
        smoothed_DOA_deg_m(t, :) = current_smoothed_doa;

        % --- Update Plots ---
        % Check if the figure window is still open before updating plots.
        if ishandle(live_fig)
            % --- Update Spectrum Plot (Subplot 1) ---
            subplot(2, 1, 1); % Activate the first subplot
            % Update the spectrum data if available.
            if ~isempty(xaxis) && ~isempty(sp_dB1)
                set(h_spectrum1, 'XData', xaxis, 'YData', sp_dB1); % Update primary spectrum
                % Update the second spectrum plot only if using OP3R_AZ and data exists.
                if strcmp(algorithm, 'OP3R_AZ') && ~isempty(sp_dB2)
                    set(h_spectrum2, 'XData', xaxis, 'YData', sp_dB2, 'Visible', 'on');
                else
                    set(h_spectrum2, 'Visible', 'off'); % Hide if not OP3R_AZ or no data
                end
            else
                 % Clear spectrum plots if no data was generated.
                 set(h_spectrum1, 'XData', NaN, 'YData', NaN);
                 set(h_spectrum2, 'Visible', 'off');
            end
            % Update the title with the current trial number.
            title_str_spec_update = sprintf('%s Spatial Spectrum - Trial %d of %d', algorithm, t, numtrials);
            title(title_str_spec_update);

            % --- Update Tracking Plot (Subplot 2) ---
            subplot(2, 1, 2); % Activate the second subplot
            valid_trials = 1:t; % Define the range of trials completed so far
            % Update the plot data for each source line.
            for src = 1:num_sources
                 set(h_doa_lines(src), 'XData', valid_trials, 'YData', smoothed_DOA_deg_m(valid_trials, src));
            end
            % Update the title with the current trial number.
            title_str_track_update = sprintf('Smoothed DOA Tracking History (alpha=%.1f) - Trial %d', alpha, t);
            title(title_str_track_update);

            % Refresh the figure window to display the updates.
            drawnow limitrate; % Use limitrate for potentially smoother updates without overwhelming CPU.
        else
            % If the figure window was closed by the user, stop the loop.
            disp('Live plot figure closed. Stopping loop.');
            break;
        end

        num_error = 0; % Reset the consecutive error counter upon successful trial completion.

    catch ME
        % Catch any errors occurring within the main loop (acquisition, processing, plotting).
        fprintf("Error in trial %d (%s): %s\n", t, ME.identifier, ME.message);

        % Handle specific hardware/connection related errors.
        if contains(ME.identifier, {'adi:', 'MATLAB:UndefinedFunction','comm:', 'instrument:', 'MATLAB:hwconnectinstaller'})
             num_error = num_error + 1; % Increment consecutive error counter
             fprintf('Consecutive hardware/connection error count: %d\n', num_error);
             if num_error >= max_errors
                 % If max errors exceeded, abort the script.
                 disp("Exceeded maximum error count for hardware/connection issues. Aborting.");
                 % Attempt to release hardware before rethrowing the error.
                 if ~isempty(rx_board1) && isvalid(rx_board1), rx_board1.release(); end
                 if ~isempty(rx_board2) && isvalid(rx_board2), rx_board2.release(); end
                 rethrow(ME); % Rethrow the caught error to stop execution.
             end
             pause(1); % Pause briefly before the next trial attempt.
        else % Handle other non-recoverable errors (e.g., algorithm errors).
             disp("Non-recoverable error encountered. Aborting.");
             % Attempt to release hardware before rethrowing the error.
             if ~isempty(rx_board1) && isvalid(rx_board1), rx_board1.release(); end
             if ~isempty(rx_board2) && isvalid(rx_board2), rx_board2.release(); end
             rethrow(ME); % Rethrow the caught error to stop execution.
        end

         % Mark estimates for the failed trial as NaN.
         smoothed_DOA_deg_m(t, :) = NaN;
         % NOTE: Retrying the same trial (t = t - 1) is commented out to avoid potential infinite loops
         % if the error condition persists. The loop will proceed to the next trial number.
         % t = t - 1;
    end
end

%% Release Hardware Resources
% Ensure hardware resources are released cleanly after the loop finishes or if an error occurred.
disp('Releasing hardware resources...');
if ~isempty(rx_board1) && isvalid(rx_board1)
    rx_board1.release();
end
if ~isempty(rx_board2) && isvalid(rx_board2)
    rx_board2.release();
end
disp('Hardware released.');

%% Helper Functions Section - REMOVED

% NOTE: The following helper functions are assumed to be available as
%       separate .m files on the MATLAB path:
%       - get_data_fmc5_OptB.m (which likely calls poll_fmc5.m)
%       - reduce_snapshots.m
%       - sjp_get_doa_MUSIC_QR.m
%       - sjp_get_doa_RQR_search_az.m
%       - sjp_get_doa_OP3R_AZ_8.m

