% ======================================================================= %
% Script:   MusicAlg_v2.m
% Function: Version two performs updated MUSIC Algorithm w/ svd
%           decomposition instead of standard eigvalue decomp
% Notes:    
% Author:   Parker Reeves
% Date:     04/08/2026
% Inputs:
%   raw_data    - input matrix data directly from antenna array
%   J           - identity matrix formed for forward-backward averaging
%
% Outputs:
%   En          - Noise subspace
% ======================================================================= %

function [En] = MusicAlg_v3(raw_data,J)

raw_data = raw_data';
R_x = (raw_data * raw_data') / size(raw_data, 2);

R_fb = [R_x, J * conj(R_x)];
%R_fb = 0.5 * (R_x + J * conj(R_x) * J)

[~, ~,  U_x] = svd(R_q, 'econ');

En = U_x(:, 2:end);