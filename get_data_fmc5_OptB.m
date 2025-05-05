% GET_DATA_FMC5_OPTB Acquires and combines data from two FMComms5 boards for an 8-element ULA.
%
% Purpose:
%   This function orchestrates data acquisition from two separate FMComms5
%   receiver boards, intended to operate as a combined 8-element Uniform
%   Linear Array (ULA). It calls the `poll_fmc5` function for each board
%   and structures the combined data according to the physical layout
%   assumed by the DOA estimation algorithms.
%
% Inputs:
%   rx_board1  - MATLAB adi.FMComms5.Rx object configured for the first board
%                (e.g., representing elements -4 to -1 of the ULA).
%   rx_board2  - MATLAB adi.FMComms5.Rx object configured for the second board
%                (e.g., representing elements 0 to +3 of the ULA).
%   NUM_RETRY  - Integer specifying the maximum number of retry attempts
%                `poll_fmc5` should make if data acquisition fails for either board.
%
% Output:
%   rx_m       - Complex double matrix (8 x N) containing the combined received data.
%                N is the number of samples per frame defined during receiver setup.
%                Rows 1-4: Data from rx_board1, with row order flipped.
%                Rows 5-8: Data from rx_board2, in original channel order.
%
% Key Operations:
%   1. Calls `poll_fmc5` to retrieve data from `rx_board1`.
%   2. **Flips the row order** of the data from `rx_board1`. This critical step
%      reorders the channels from board 1 to match the assumed spatial ULA
%      geometry (e.g., if board 1 channels physically map to elements -1, -2, -3, -4,
%      flipping arranges them as -4, -3, -2, -1 in the matrix rows).
%   3. Calls `poll_fmc5` to retrieve data from `rx_board2`.
%   4. Vertically concatenates the flipped data from board 1 and the data from
%      board 2 to form the final 8-channel data matrix `rx_m`.
%
% Dependencies:
%   - poll_fmc5.m: Must be available on the MATLAB path.
%   - MATLAB Support Package for Analog Devices ADALM-PLUTO Radio (internally uses libiio).

function [rx_m] = get_data_fmc5_OptB(rx_board1, rx_board2, NUM_RETRY)

    % Acquire data from board1 and flip the row order
    data_board1 = flip(poll_fmc5(rx_board1, NUM_RETRY), 1);
    
    % Acquire data from board2 (no flipping needed)
    data_board2 = poll_fmc5(rx_board2, NUM_RETRY);
    
    % Combine the two sets of data vertically (resulting in an 8xN matrix)
    rx_m = [data_board1; data_board2];
end