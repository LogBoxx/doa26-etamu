% ======================================================================= %
% Script:   MusicAlg.m
% Function: Performs MUSIC algorithm for either ULA or UCA
% Notes:    
% Author:   Parker Reeves
% Date:     02/24/2026
% Inputs:
%   raw_data    - input matrix data directly from antenna array
%   J           - identity matrix formed for forward-backward averaging
%
% Outputs:
%   En          - Noise subspace
% ======================================================================= %

function [En] = MusicAlg(raw_data,J)

raw_data = raw_data';
R_x = (raw_data * raw_data') / size(raw_data, 2);

R_fb = 0.5 * (R_x + J * conj(R_x) * J);

[Evec, Eval] = eig(R_fb);
[~, idx] = sort(diag(Eval), 'descend');
Evec = Evec(: , idx);
En = Evec(: , 2:end);