%% MATLAB ULA SIMULATION Q5

clc; clear; close all;

%% SIMULATION SETUP
M = 10;                
d = 0.0625;      
n_snapshots = 200;       
theta_true = [60 75 85];
c = 3e8;                  
f = 2.41e9;               
lambda = c/f;             
k = 2*pi/lambda;          
delta = d*lambda;         
SNR_dB = 10;             
n_trials = 300;         

est_DOAs_all = [];

%% ALGORITHM
for mc = 1:n_trials

    A = exp(-1j*k*(0:M-1)'*delta*sind(theta_true));

    s_sig = randn(length(theta_true),n_snapshots) + 1j*randn(length(theta_true),n_snapshots);
    x = A*s_sig;

    noise = (randn(M,n_snapshots) + 1j*randn(M,n_snapshots))/sqrt(2) * 10^(-SNR_dB/20);
    x_noisy = x + noise;

    R = (x_noisy*x_noisy')/n_snapshots;

    [E,D] = eig(R);
    [~,idx] = sort(diag(D),'descend');
    E = E(:,idx);
    En = E(:,length(theta_true)+1:end);

    theta_scan = 0:0.5:90;
    Pmusic = zeros(size(theta_scan));
    for i = 1:length(theta_scan)
        a_theta = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(i)));
        Pmusic(i) = 1/(a_theta'*(En*En')*a_theta);
    end

    Pvals = abs(Pmusic);
    est_DOAs = [];
    for i = 2:length(Pvals)-1
        if Pvals(i) > Pvals(i-1) && Pvals(i) > Pvals(i+1)
            est_DOAs(end+1) = theta_scan(i);
        end
    end

    if length(est_DOAs) >= length(theta_true)
        [~, order] = sort(Pvals(ismember(theta_scan,est_DOAs)), 'descend');
        est_DOAs = est_DOAs(order(1:length(theta_true)));
    end

    est_DOAs_all = [est_DOAs_all est_DOAs];
end

%% HISTOGRAM PLOT
figure;
histogram(est_DOAs_all, 'BinWidth', 1, 'Normalization', 'pdf');
hold on;
xline(theta_true(1),'r--','LineWidth',1.5);
xline(theta_true(2),'g--','LineWidth',1.5);
xline(theta_true(3),'b--','LineWidth',1.5);
xlabel('Estimated DOA (degrees)');
ylabel('Probability Density');
title('Histogram of Estimated DOAs for Multiple Sources (60°, 75°, 85°)');
legend('Estimated DOAs','True DOAs');
xlim([0 90]);
grid on;
