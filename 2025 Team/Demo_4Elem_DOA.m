%% Demo_4Elem_DOA.m
% Main script for azimuth-only DOA estimation using a 4-element ULA.
%
% This script:
%   1. Sets up receivers for data acquisition.
%   2. Acquires data using get_data_fmc5_4elem.
%   3. Reduces raw samples to the desired number of snapshots.
%   4. Calls the selected DOA estimation algorithm.
%       Options:
%         - 'MUSIC_QR'     : MUSIC-like approach with QR decomposition.
%         - 'RQR'          : Rank-Revealing QR Search algorithm.
%         - 'ESPRIT'       : ESPRIT-like approach.
%   5. Updates live plots of the spatial spectrum and DOA estimates.
%   6. Displays a final scatter plot of the estimated DOA.
%
% Ensure that get_data_fmc5_4elem.m, reduce_snapshots.m, music_qr_4elem.m, 
% rqr_search_4elem.m, and esprit_4elem.m are in your working directory.

clear; clc; close all;

%% User Options
% Choose algorithm: 'MUSIC_QR', 'RQR', or 'ESPRIT'
algorithm = 'RQR';  % Change as desired

num_sources = 1;
dtheta_deg = 0.05;
numtrials = 100;

% Added parameter for controlling number of snapshots
num_snapshots = 15;  % Recommended for low-snapshot methods like MUSIC_QR

% Set center frequency and sampling parameters
CenterFrequency = 2.4e9;
SamplingRate = 30e6;        % 30 MHz
SamplesPerFrame = 2^14;     % 16384 samples per frame

max_reconnect = 2;  % Maximum number of retry attempts for data acquisition
max_errors = 5;     % Maximum number of consecutive errors allowed
num_error = 0;

%% Receiver Setup for 4-Element Array
% Setup for receiver hardware - this will depend on your specific hardware
% This example assumes a single receiver device with 4 channels
try
    % Example for a 4-channel FMComms5 setup (adjust according to your hardware)
    rx_device = adi.FMComms5.Rx('uri','ip:192.168.0.1', ...
        'CenterFrequency', CenterFrequency, ...
        'SamplingRate', SamplingRate, 'SamplesPerFrame', SamplesPerFrame);
    rx_device.EnabledChannels = [1 2 3 4];  % Enable all 4 channels
catch ME
    warning('Hardware setup failed. Running in simulation mode.');
    fprintf('Error: %s\n', ME.message);
    simulation_mode = true;
end

% Check if we should run in simulation mode
if ~exist('simulation_mode', 'var')
    simulation_mode = true;
end

if ~exist('figNum', 'var')
    figNum = 1;
end

%% Preallocate Array for DOA Estimates and Setup Plot Grid
az_DOA_deg_m = zeros(numtrials, num_sources);
xaxis_sp_v = linspace(-90, 90, 180/dtheta_deg + 1);

%% Main Loop: Data Acquisition and DOA Estimation
for t = 1:numtrials
    try
        % Acquire data (either from hardware or simulation)
        if simulation_mode
            % Generate simulated signal for testing
            % Example: Single source at 30 degrees
            rx_m = generate_simulated_data(4, 1000, [30], 10);
        else
            % Acquire data from hardware using get_data_fmc5_4elem function
            rx_m = get_data_fmc5_4elem(rx_device, max_reconnect);
        end
        
        % Reduce the raw samples to the desired number of snapshots
        rx_snapshots = reduce_snapshots(rx_m, num_snapshots);
        
        % Call the selected DOA estimation algorithm
        switch algorithm
            case 'ESPRIT'
                [az_DOA_deg, sp_board1_db_v, sp_board2_db_v] = esprit_4elem(rx_snapshots, num_sources, dtheta_deg);
                update_live_plot(algorithm, t, az_DOA_deg, xaxis_sp_v, sp_board1_db_v, sp_board2_db_v, num_snapshots);
            case 'RQR'
                [az_DOA_deg, sp_dB, xaxis] = rqr_search_4elem(rx_snapshots, num_sources, dtheta_deg);
                update_live_plot(algorithm, t, az_DOA_deg, xaxis, sp_dB, num_snapshots);
            case 'MUSIC_QR'
                [az_DOA_deg, sp_dB, xaxis] = music_qr_4elem(rx_snapshots, num_sources, dtheta_deg);
                update_live_plot(algorithm, t, az_DOA_deg, xaxis, sp_dB, num_snapshots);
            otherwise
                error('Unrecognized algorithm option.');
        end
        
        % Store the estimated azimuth DOA for final statistics
        az_DOA_deg_m(t, :) = az_DOA_deg;
        
        % Print current estimate to console
        fprintf('Trial %d: Estimated DOA = %0.2f degrees\n', t, az_DOA_deg);
        
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

%% Release Hardware Resources
if ~simulation_mode
    rx_device.release();
end

%% Final Plot: Scatter Plot of DOA Estimates
figure(figNum);
est_std = std(az_DOA_deg_m(:));
scatter(az_DOA_deg_m, zeros(size(az_DOA_deg_m)), 'x', 'LineWidth', 1);
title(["Azimuth-Only DOA (4-Element ULA)", "Std. = " + num2str(est_std), ...
       "Mean = " + num2str(mean(az_DOA_deg_m(:))) + " deg"]);
xlabel("Azimuth (deg)");
ylabel("Elevation (not used)");
xlim([-90, 90]);
ylim([-10, 10]);
grid on;
figNum = figNum + 1;

%% Consolidated Live Plotting Function
function update_live_plot(algorithm, t, az_DOA_deg, varargin)
    % Get the number of snapshots (last argument)
    num_snapshots_arg = varargin{end};
    % Universal plotting function for DOA algorithms
    % Parameters:
    %   algorithm - String identifying which algorithm is in use
    %   t - Current trial number
    %   az_DOA_deg - Estimated DOA in degrees
    %   varargin - Algorithm-specific spectrum data:
    %     For ESPRIT: {xaxis_sp_v, sp_board1_db_v, sp_board2_db_v}
    %     For RQR/MUSIC_QR: {xaxis, sp_dB}
    
    figure(100); clf;  % Use a consistent figure number for live plotting
    
    % Plot spectrum based on the algorithm
    if strcmp(algorithm, 'ESPRIT')
        xaxis_sp_v = varargin{1};
        sp_board1_db_v = varargin{2};
        sp_board2_db_v = varargin{3};
        % num_snapshots_arg is varargin{4}
        
        % Combined spectrum plot
        subplot(2,1,1);
        plot(xaxis_sp_v, sp_board1_db_v, 'b', 'LineWidth', 1.5); hold on;
        plot(xaxis_sp_v, sp_board2_db_v, 'r', 'LineWidth', 1.5);
        xlabel("Azimuth (deg)");
        ylabel("Spectrum (dB)");
        title("DOA Spectra (Blue: Subarray 1, Red: Subarray 2)");
        xlim([-90, 90]); ylim([-30, 0]);
        grid on; legend('Subarray 1', 'Subarray 2');
    else % RQR or MUSIC_QR
        xaxis = varargin{1};
        sp_dB = varargin{2};
        % num_snapshots_arg is varargin{3}
        
        % Full spatial spectrum plot
        subplot(2,1,1);
        plot(xaxis, sp_dB, 'm', 'LineWidth', 2);
        xlabel("Azimuth (deg)");
        ylabel("Spatial Spectrum (dB)");
        title("Full-Array Spatial Spectrum");
        xlim([-90, 90]); ylim([-30, 0]);
        grid on;
    end
    
    % DOA estimate plot - common for all algorithms
    subplot(2,1,2);
    scatter(az_DOA_deg, zeros(size(az_DOA_deg)), 300, 'x', 'LineWidth', 4);
    title("DOA Estimate, Trial: " + num2str(t));
    xlabel("Azimuth (deg)"); ylabel("Elevation (not used)");
    xlim([-90, 90]); ylim([-10, 10]); grid on;
    
    % Use the explicitly passed number of snapshots
    sgtitle("Azimuth-Only DOA (" + algorithm + ") - Using " + num2str(num_snapshots_arg) + " snapshots");
    drawnow;  % Ensure immediate plot update
end

% Function to generate simulated data for testing without hardware
function rx_m = generate_simulated_data(num_antennas, num_samples, source_angles, snr_db)
    % Generates simulated data for a ULA with specified source directions
    % Inputs:
    %   num_antennas: Number of antennas in the ULA
    %   num_samples: Number of time samples to generate
    %   source_angles: Vector of source angles in degrees
    %   snr_db: Signal-to-noise ratio in dB
    
    % Parameters
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
    
    % Add noise based on SNR
    signal_power = mean(abs(signal_component(:)).^2);
    noise_power = signal_power / (10^(snr_db/10));
    noise = sqrt(noise_power/2) * (randn(num_antennas, num_samples) + 1j*randn(num_antennas, num_samples));
    
    % Final received signal
    rx_m = signal_component + noise;
end

% Function to acquire data from the 4-element array hardware
function [rx_m] = get_data_fmc5_4elem(rx_device, NUM_RETRY)
% get_data_fmc5_4elem
%   Retrieves data from a 4-channel receiver and processes it for DOA estimation.
%
%   Inputs:
%       rx_device  - Receiver object with 4 enabled channels
%       NUM_RETRY  - Maximum number of retry attempts for data acquisition
%
%   Output:
%       rx_m       - Data matrix (4 x N samples)

    valid = false;
    j = 0; % Tracks # of times data has been retrieved (but has not been valid)
    k = 0; % Tracks # of connection attempts
    
    while ~valid
        try
            % Acquire data from the device
            raw_data = rx_device();
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
