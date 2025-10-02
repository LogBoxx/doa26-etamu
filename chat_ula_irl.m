%% SIMULATION WITH MONTE CARLO TRIALS (ULA Elevation 0°–90°)
clear; clc; close all;
% 
% c  = 3e8; 
% f  = 2.409e9; 
% lambda = c/f; 
% k  = 2*pi/lambda;
% 
% N  = 4;                  % number of elements
% d  = lambda/2;           % element spacing (ULA, half-wavelength)
% 
% theta0_deg = 30;         % true elevation angle (deg), range 0:90
% theta0 = deg2rad(theta0_deg);
% 
% M = 15;                  % snapshots per trial
% SNRdB = 5;               % per-sensor SNR
MCtrials = 15;          % Monte Carlo runs
rx = adi.FMComms5.Rx('uri','ip:192.168.0.1');
rx.EnabledChannels = [1 2 3 4];
% % Grid for pseudospectrum: 0–90 deg
% thetas = linspace(0, pi/2, 721);   
% PM_accum = zeros(size(thetas));
% 
% % Store estimated DoA of each trial
% DoA_estimates = zeros(1,MCtrials);
% 
% %% Steering vector for ULA
% steer = @(theta) exp(-1j * k * d * (0:N-1).' * sin(theta));

%% Monte Carlo trials
for mc = 1:MCtrials
    
    % Source symbols
    % s = (randn(1,M) + 1j*randn(1,M)) / sqrt(2);  % unit power
    % A = steer(theta0);                           % steering vector (4x1)
    % 
    % % Received signal with noise
    % X = A*s + db2mag(-SNRdB)*(randn(N,M)+1j*randn(N,M))/sqrt(2);
    % 
    % Sample covariance matrix
    R = (rx()*rx()')/M;
    
    % MUSIC (assume 1 source)
    [U,D] = eig((R+R')/2);              
    [~,idx] = sort(diag(D), 'descend');
    Un = U(:, idx(2:end));              % noise subspace
    
    % MUSIC pseudospectrum
    PM = zeros(size(thetas));
    for i=1:numel(thetas)
        atheta = steer(thetas(i));
        PM(i) = 1 / (norm(Un' * atheta)^2 + 1e-12);
    end
    
    % Estimate DoA from this trial
    [~,iMU] = max(PM); 
    theta_mu = thetas(iMU);
    theta_mu_deg = rad2deg(theta_mu);
    
    DoA_estimates(mc) = theta_mu_deg;
    
    % Accumulate spectrum for averaging
    PM_accum = PM_accum + PM;
end

%% Average pseudospectrum
PM_avg = PM_accum / MCtrials;
PM_avg = PM_avg / max(PM_avg);

%% Print summary
fprintf('True elevation   = %6.2f deg\n', theta0_deg);
fprintf('Mean DoA (MUSIC) = %6.2f deg\n', mean(DoA_estimates));
fprintf('Std Dev          = %6.2f deg\n', std(DoA_estimates));

%% Plot averaged MUSIC spectrum
figure;
plot(rad2deg(thetas), PM_avg, 'LineWidth', 1.8);
xline(theta0_deg, '--k', 'LineWidth',1.5);
xlabel('Elevation angle (deg)'); ylabel('Averaged MUSIC spectrum');
legend('MUSIC spectrum (avg)','True elevation','Location','best');
grid on;
title(sprintf('MUSIC DoA Monte Carlo Avg (%d trials, 4-ele ULA @ 2.4 GHz)', MCtrials));

%% Plot histogram of DoA estimates
figure;
histogram(DoA_estimates, 0:2:90, 'Normalization','pdf');
xline(theta0_deg,'--k','LineWidth',1.5);
xlabel('Estimated DoA (deg)'); ylabel('PDF');
grid on;
title(sprintf('Histogram of DoA Estimates (%d trials, SNR=%d dB)', MCtrials,SNRdB));

%% Plot scatter of estimates vs trial index
figure;
stem(1:MCtrials, DoA_estimates, 'filled');
yline(theta0_deg,'--k','LineWidth',1.5);
xlabel('Trial'); ylabel('Estimated DoA (deg)');
grid on;
title('DoA Estimate per Trial');

%% Utility
function y = db2mag(x), y = 10.^(x/20); end
