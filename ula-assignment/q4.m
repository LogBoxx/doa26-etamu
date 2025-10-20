%% MATLAB ULA SIMULATION Q4

clc; clear; close all;

%% SIMULATION SETUP
M = 10;                
d = 0.0625;        
theta_true = 55;   
c = 3e8;                   
f = 2.41e9;                
lambda = c/f;              
k = 2*pi/lambda;           
delta = d*lambda;          
SNR_dB = 5;            
n_trials = 300;         
snap_list = 100:100:1000; 
std_est = zeros(size(snap_list));

%% ALGORITHM
for s_idx = 1:length(snap_list)
    n_snapshots = snap_list(s_idx);     
    est_DOAs = zeros(1,n_trials);

    for mc = 1:n_trials
        a = exp(-1j*k*(0:M-1)'*delta.*sind(theta_true));
        s_sig = randn(1,n_snapshots) + 1j*randn(1,n_snapshots);
        x = a*s_sig;

        noise = (randn(M,n_snapshots) + 1j*randn(M,n_snapshots))/sqrt(2)*10^(-SNR_dB/20);
        x_noisy = x + noise;

        R = (x_noisy*x_noisy')/n_snapshots;

        [E,D] = eig(R);
        [~,idx] = sort(diag(D),'descend');
        E = E(:,idx);
        En = E(:,2:end);

        theta_scan = 0:0.5:90;
        Pmusic = zeros(size(theta_scan));
        for i = 1:length(theta_scan)
            a_theta = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(i)));
            Pmusic(i) = 1/(a_theta'*(En*En')*a_theta);
        end

        [~,idxMax] = max(abs(Pmusic));
        est_DOAs(mc) = theta_scan(idxMax);
    end
    
    std_est(s_idx) = std(est_DOAs);
end

%% PLOT RESULTS
figure;
plot(snap_list, std_est,'-o','LineWidth',1.5);
xlabel('Number of Snapshots');
ylabel('Standard Deviation (degrees)');
title('Standard Deviation vs Number of Snapshots');
grid on;
