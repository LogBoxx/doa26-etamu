%% SIMULATION WITH MONTE CARLO TRIALS
clear; clc; close all;

c  = 3e8; 
f  = 2.409e9; 
lambda = c/f; 
k  = 2*pi/lambda;
a  = 0.044;
N  = 4;
phin = (0:N-1).' * (2*pi/N);

phi0_deg = 0;    % true azimuth (deg), range -180:180
phi0 = deg2rad(phi0_deg);

M = 15;               % snapshots per trial
SNRdB = 5;            % per-sensor SNR
MCtrials = 150;       % Monte Carlo runs

% Grid for pseudospectrum
phis = linspace(-pi, pi, 1081);   % -180..180 deg
PM_accum = zeros(size(phis));

% Store estimated DoA of each trial
DoA_estimates = zeros(1,MCtrials);

%% Steering vector for UCA
steer = @(phi) exp(-1j * k * a * cos(phi - phin));

%% Monte Carlo trials
for mc = 1:MCtrials
    
    % Source symbols
    s = (randn(1,M) + 1j*randn(1,M)) / sqrt(2);  % unit power
    A = steer(phi0);                             % steering vector (4x1)
    
    % Received signal with noise
    X = A*s + db2mag(-SNRdB)*(randn(N,M)+1j*randn(N,M))/sqrt(2);
    
    % Sample covariance matrix
    R = (X*X')/M;
    
    % MUSIC (assume 1 source)
    [U,D] = eig((R+R')/2);              
    [~,idx] = sort(diag(D), 'descend');
    Un = U(:, idx(2:end));              % noise subspace
    
    % MUSIC pseudospectrum
    PM = zeros(size(phis));
    for i=1:numel(phis)
        aphi = steer(phis(i));
        PM(i) = 1 / (norm(Un' * aphi)^2 + 1e-12);
    end
    
    % Estimate DoA from this trial
    [~,iMU] = max(PM); 
    phi_mu = phis(iMU);
    phi_mu_deg = rad2deg(angle(exp(1j*phi_mu))); % wrap [-180,180]
    
    DoA_estimates(mc) = phi_mu_deg;
    
    % Accumulate spectrum for averaging
    PM_accum = PM_accum + PM;
end

%% Average pseudospectrum
PM_avg = PM_accum / MCtrials;
PM_avg = PM_avg / max(PM_avg);

%% Print summary
fprintf('True azimuth     = %6.2f deg\n', phi0_deg);
fprintf('Mean DoA (MUSIC) = %6.2f deg\n', mean(DoA_estimates));
fprintf('Std Dev          = %6.2f deg\n', std(DoA_estimates));

%% Plot averaged MUSIC spectrum
figure;
plot(rad2deg(phis), PM_avg, 'LineWidth', 1.8);
xline(phi0_deg, '--k', 'LineWidth',1.5);
xlabel('Azimuth angle (deg)'); ylabel('Averaged MUSIC spectrum');
legend('MUSIC spectrum (avg)','True azimuth','Location','best');
grid on;
title(sprintf('MUSIC DoA Monte Carlo Avg (%d trials)', MCtrials));

%% Plot histogram of DoA estimates
figure;
histogram(DoA_estimates, -180:5:180, 'Normalization','pdf');
xline(phi0_deg,'--k','LineWidth',1.5);
xlabel('Estimated DoA (deg)'); ylabel('PDF');
grid on;
title(sprintf('Histogram of DoA Estimates (%d trials, SNR=%d dB)', MCtrials,SNRdB));

%% Plot scatter of estimates vs trial index
figure;
stem(1:MCtrials, DoA_estimates, 'filled');
xline([0 MCtrials],[phi0_deg phi0_deg],'--k','LineWidth',1.5);
xlabel('Trial'); ylabel('Estimated DoA (deg)');
grid on;
title('DoA Estimate per Trial');

%% Utility
function y = db2mag(x), y = 10.^(x/20); end

