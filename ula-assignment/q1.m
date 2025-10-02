%% MATLAB ULA SIMULATION Q1

clc; clear; close;

%% SIMULATION SETUP
M = 4;                    
d = 0.0625;
n_snapshots = 200;               
theta_true = 30;    
c = 3e8;
f = 2.41e9;
lambda = c/f;
k = 2*pi/lambda;
delta = d*lambda;
SNR_dB = 0:5:30;
n_trials = 300;

std_est  = zeros(size(SNR_dB));

%% ALGORITHM
for s = 1:length(SNR_dB)
    est_DOAs = zeros(1,n_trials);
    for mc = 1:n_trials

        a = exp(-1j*k*(0:M-1)'*delta.*sind(theta_true));
        
        s_sig = randn(1,n_snapshots) + 1j*randn(1,n_snapshots);
        x = a*s_sig;
        
        noise = (randn(M,n_snapshots) + 1j*randn(M,n_snapshots))/sqrt(2)*10^(-SNR_dB(s)/20);
        x_noisy = x + noise;
        
        R = (x_noisy*x_noisy')/n_snapshots;
        
        [E,D] = eig(R);
        [~,idx] = sort(diag(D),'descend');
        E = E(:,idx);
        En = E(:,2:end);
        
        theta_scan = -90:1:90;
        Pmusic = zeros(size(theta_scan));
        for i = 1:length(theta_scan)
            a_theta = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(i)));
            Pmusic(i) = 1/(a_theta'*(En*En')*a_theta);
        end
        [~,idxMax] = max(abs(Pmusic));
        est_DOAs(mc) = theta_scan(idxMax);
    end
    std_est(s)  = std(est_DOAs);
end

%% PLOT PSEUDOSPECTRUM
figure; 
plot(SNR_dB, std_est,'-o','LineWidth',1.5);
xlabel('SNR (dB)'); ylabel('Standard Deviation (degrees)');
ylim([0,90])
title('Standard Deviation vs SNR'); grid on;