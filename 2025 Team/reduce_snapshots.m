% REDUCE_SNAPSHOTS Reduces the number of time samples (snapshots) in received data.
%
% Purpose:
%   Takes a matrix of raw received data (Channels x Samples) and reduces the
%   number of samples (columns) down to a specified target number (`num_snapshots`).
%   This is often done in array processing to reduce computational complexity
%   of subsequent algorithms (like covariance matrix estimation) and can sometimes
%   improve performance by averaging out noise over short time segments.
%
% Method Used: Segment Averaging
%   The function divides the total number of input samples into `num_snapshots`
%   segments of approximately equal length. It then calculates the mean of the
%   samples within each segment for each channel. The resulting means form the
%   columns of the output matrix.
%
% Inputs:
%   rx_data       - Complex double matrix (K x N_raw) containing the raw
%                   received data. K is the number of channels (antennas),
%                   N_raw is the original number of samples per channel.
%   num_snapshots - Integer specifying the desired number of snapshots (columns)
%                   in the output matrix. Must be less than or equal to N_raw.
%
% Output:
%   reduced_data  - Complex double matrix (K x num_snapshots) containing the
%                   data after snapshot reduction via segment averaging.
%
% Error Handling:
%   - Throws an error if the number of raw samples `N_raw` is less than the
%     desired `num_snapshots`, as segment length would be less than 1.
%
% Example:
%   % Assume raw_data is 8x10000
%   num_snaps = 20;
%   reduced_snaps = reduce_snapshots(raw_data, num_snaps); % Output will be 8x20

%Brians Layman comment:
% The function divides the overall stream of (2^15) data points by N. 
% N is the snapshots we desire, it is how many data points/snapshots we want instead of the 2^15 data points 
% (each data point is a snapshots when "reduce snapshot" function is not applied) 
% and the value of that segment is automatically averaged in order to be defined as a snapshot.
% EX:
% Imagine 2^15 devided by 15 and then those 15 groups are averaged, that is
% the output of this function and the input for the DOA function

function [reduced_data] = reduce_snapshots(rx_data, num_snapshots)


    [num_antennas, num_samples] = size(rx_data);
    
    % Method 1: Segment averaging (more robust against noise)
    segment_length = floor(num_samples / num_snapshots);
    reduced_data = zeros(num_antennas, num_snapshots);
    
    for i = 1:num_snapshots
        start_idx = (i-1) * segment_length + 1;
        end_idx = min(i * segment_length, num_samples);
        reduced_data(:, i) = mean(rx_data(:, start_idx:end_idx), 2);
    end
    
    % Alternative methods (commented out):
    % Method 2: Decimation (taking evenly spaced samples)
    % indices = round(linspace(1, num_samples, num_snapshots));
    % reduced_data = rx_data(:, indices);
    
    % Method 3: Random sampling
    % indices = sort(randperm(num_samples, num_snapshots));
    % reduced_data = rx_data(:, indices);
end
