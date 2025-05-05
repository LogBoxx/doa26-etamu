function [rx_m] = poll_fmc5(RX, NUM_RETRY)
% POLL_FMC5 Acquires a single frame of data from an FMComms5 receiver object with retry.
%
% Purpose:
%   This function attempts to retrieve one frame of complex sample data from a
%   specified adi.FMComms5.Rx receiver object. It includes error handling
%   and retry logic to cope with potential communication issues or invalid data
%   reception common in SDR operations. It also transposes the output to match
%   the convention (Channels x Samples) often used in array processing.
%
% Inputs:
%   RX        - MATLAB adi.FMComms5.Rx object, already configured (center frequency,
%               sampling rate, samples per frame, enabled channels).
%   NUM_RETRY - Integer specifying the maximum number of times to attempt data
%               acquisition or reconnection if an error occurs or invalid data is received.
%
% Output:
%   rx_m_transposed - Complex double matrix (K x N) containing the received data.
%                     K is the number of enabled channels on the RX object.
%                     N is the number of samples per frame defined for the RX object.
%                     Returns an error if valid data cannot be acquired after NUM_RETRY attempts.
%
% Key Operations:
%   1. Enters a loop that continues until valid data is acquired or retry limit is exceeded.
%   2. Inside the loop:
%      a. Calls the receiver object `RX()` which triggers data acquisition from the hardware.
%         This call returns the data (N x K) and a validity flag.
%      b. Catches potential errors during the `RX()` call (e.g., connection lost).
%         If an error occurs, it pauses, increments the connection attempt counter (`k`),
%         and retries if `k` is less than `NUM_RETRY`. Throws an error if connection fails repeatedly.
%      c. Checks the `valid` flag returned by `RX()`. If data exists but is marked invalid,
%         it increments the invalid data counter (`j`) and continues the loop if `j` is less
%         than `NUM_RETRY`. Throws an error if invalid data is received repeatedly.
%   3. If valid data (`rx_m`) is successfully acquired, the loop terminates.
%   4. Transposes the acquired data matrix `rx_m` (N x K) to `rx_m_transposed` (K x N)
%      before returning.
%
% Dependencies:
%   - MATLAB Support Package for Analog Devices ADALM-PLUTO Radio (internally uses libiio).
%
% Date: March 2025
% Authors: Team TAMUC

    valid = false;

    j = 0; % Tracks # of times data has been retrieved (but has not been valid)
    k = 0; % Tracks # of connection attempts
    
    while ~valid
        try
            [rx_m, valid] = RX();
        catch ME
            disp("Retrying connection...");
            pause(1);
            k = k + 1;
            if k >= NUM_RETRY
                disp("Failed to connect to board");
                rethrow(ME);
            end
        end
        
        if (exist('rx_m', 'var') && ~valid)
            disp("Invalid data...");
            j = j + 1;
        end
        
        if ((j >= NUM_RETRY) && ~valid)
            disp("Connecting but not grabbing valid data for some reason");
            RX.release();
            error('Failed to acquire valid data after multiple attempts');
        end
    end

    rx_m = rx_m';
end
