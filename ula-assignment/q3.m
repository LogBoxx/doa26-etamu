%% MATLAB ULA SIMULATION Q3

clc; clear; close all;

%% SIMULATION SETUP
theta_true   = 50;
SNR_dB       = 10;            
n_snapshots  = 200;          
n_trials     = 300;       
c = 3e8;                   
f = 2.41e9;                
lambda = c/f;                 
d = 0.0625;               
k = 2*pi/lambda;
M_values = 4:2:24;          
std_est  = zeros(size(M_values));

%% ALGORITHM
for m_idx = 1:length(M_values)
    M = M_values(m_idx);  
    delta = d*lambda;      
    est_DOAs = zeros(1,n_trials);

    for mc = 1:n_trials
        a = exp(-1j*k*(0:M-1)'*delta*sind(theta_true));

        s_sig = randn(1,n_snapshots) + 1j*randn(1,n_snapshots);
        x = a*s_sig;

        noise = (randn(M,n_snapshots) + 1j*randn(M,n_snapshots))/sqrt(2)*10^(-SNR_dB/20);
        x_noisy = x + noise;

        R = (x_noisy*x_noisy')/n_snapshots;

        [E,D] = eig(R);
        [~,idx] = sort(diag(D),'descend');
        E = E(:,idx);
        En = E(:,2:end);

        theta_scan = 0:1:90;
        Pmusic = zeros(size(theta_scan));
        for i = 1:length(theta_scan)
            a_theta = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(i)));
            Pmusic(i) = 1/(a_theta'*(En*En')*a_theta);
        end

        [~,idxMax] = max(abs(Pmusic));
        est_DOAs(mc) = theta_scan(idxMax);
    end

    std_est(m_idx) = std(est_DOAs);
end

%% PLOT RESULTS
figure;
plot(M_values, std_est,'-o','LineWidth',1.5);
xlabel('Number of Antennas (M)');
ylabel('Standard Deviation (degrees)');
title('Standard Deviation vs Number of Antennas');
grid on;
