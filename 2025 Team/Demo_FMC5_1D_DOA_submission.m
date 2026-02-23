%% Demo_FMC5_1D_DOA_submission.m
% Generalized main script for azimuth-only DOA estimation using two FMComms5 boards
% configured as an 8-element ULA (4 channels per board).
%
% This script:
%   1. Sets up the FMComms5 receivers for two boards
%   2. Acquires data using get_data_fmc5
%   3. Reduces raw samples to the desired number of snapshots
%   4. Calls the selected DOA estimation algorithm:
%       Options:
%         - 'OP3R_AZ'      : ESPRIT-like method with separate board spectra
%         - 'RQR'          : Rank-Revealing QR Search algorithm
%         - 'MUSIC_QR'     : MUSIC-like approach for low snapshots
%   5. Updates live plots of the spatial spectrum and DOA estimates
%   6. Displays a final scatter plot of the estimated DOA
%
% Authors: Team TAMUC/ETAMU
% Date: March 2025

clear; clc; close all;

%% User Options
% Choose algorithm: 'OP3R_AZ', 'RQR', or 'MUSIC_QR'
algorithm = 'RQR';  % Change as desired

num_sources = 1;
dtheta_deg = 0.05;
numtrials = 100;

% Parameter for controlling number of snapshots
num_snapshots = 15;  

CenterFrequency = 2.4e9;
SamplingRate = 30e6;        % 30 MHz
SamplesPerFrame = 2^15;     % 32768 samples per frame

max_reconnect = 2;  % Maximum number of retry attempts for data acquisition
max_errors = 5;     % Maximum number of consecutive errors allowed
num_error = 0;

%% FMComms5 Receiver Setup for Two Boards
% Board 1 (channels 1-4)
rx_board1 = adi.FMComms5.Rx('uri','ip:192.168.1.1', ...
    'CenterFrequency', CenterFrequency, 'CenterFrequencyChipB', CenterFrequency, ...
    'SamplingRate', SamplingRate, 'SamplesPerFrame', SamplesPerFrame);
rx_board1.EnabledChannels = [1 2 3 4];

% Board 2 (channels 1-4)
rx_board2 = adi.FMComms5.Rx('uri','ip:192.168.0.1', ...
    'CenterFrequency', CenterFrequency, 'CenterFrequencyChipB', CenterFrequency, ...
    'SamplingRate', SamplingRate, 'SamplesPerFrame', SamplesPerFrame);
rx_board2.EnabledChannels = [1 2 3 4];

if ~exist('figNum','var')
    figNum = 1;
end

%% Preallocate Array for DOA Estimates and Setup Plot Grid
az_DOA_deg_m = zeros(numtrials, num_sources);
% Define a grid for live plotting for algorithms that require it:
xaxis_sp_v = linspace(-90, 90, 180/dtheta_deg + 1);

%% Main Loop: Data Acquisition and DOA Estimation
for t = 1:numtrials
    try
        % Acquire data from both boards (returns an 8xN matrix)
        rx_m = get_data_fmc5_OptB(rx_board1, rx_board2, max_reconnect);
        
        % Reduce the raw samples to the desired number of snapshots
        rx_snapshots = reduce_snapshots(rx_m, num_snapshots);
        
        % Call the selected DOA estimation algorithm
        switch algorithm
            case 'OP3R_AZ'
                [az_DOA_deg, sp_board1_db_v, sp_board2_db_v] = sjp_get_doa_OP3R_AZ_8(rx_snapshots, num_sources, dtheta_deg);
                update_live_plot(algorithm, t, az_DOA_deg, xaxis_sp_v, sp_board1_db_v, sp_board2_db_v, num_snapshots);
            case 'RQR'
                [az_DOA_deg, sp_dB, xaxis] = sjp_get_doa_RQR_search_az(rx_snapshots, num_sources, dtheta_deg);
                update_live_plot(algorithm, t, az_DOA_deg, xaxis, sp_dB, num_snapshots);
            case 'MUSIC_QR'
                [az_DOA_deg, sp_dB, xaxis] = sjp_get_doa_MUSIC_QR(rx_snapshots, num_sources, dtheta_deg);
                update_live_plot(algorithm, t, az_DOA_deg, xaxis, sp_dB, num_snapshots);
            otherwise
                error('Unrecognized algorithm option.');
        end
        
        % Store the estimated azimuth DOA for final statistics
        az_DOA_deg_m(t, :) = az_DOA_deg;
        
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
rx_board1.release();
rx_board2.release();

%% Final Plot: Scatter Plot of DOA Estimates
figure(figNum);
est_std = std(az_DOA_deg_m(:));
scatter(az_DOA_deg_m, zeros(size(az_DOA_deg_m)), 'x', 'LineWidth', 1);
title(["Azimuth-Only DOA (8-Element ULA)", "Std. = " + num2str(est_std), ...
       "Mean = " + num2str(mean(az_DOA_deg_m(:))) + " deg"]);
xlabel("Azimuth (deg)");
ylabel("Elevation (not used)");
xlim([-90, 90]);
ylim([-10, 10]);
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
    %     For OP3R_AZ: {xaxis_sp_v, sp_board1_db_v, sp_board2_db_v}
    %     For RQR/MUSIC_QR: {xaxis, sp_dB}
    
    figure(100); clf;  % Use a consistent figure number for live plotting
    
    % Plot spectrum based on the algorithm
    if strcmp(algorithm, 'OP3R_AZ')
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
        title("DOA Spectra (Blue: Board 1, Red: Board 2)");
        xlim([-90, 90]); ylim([-30, 0]);
        grid on; legend('Board 1', 'Board 2');
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
