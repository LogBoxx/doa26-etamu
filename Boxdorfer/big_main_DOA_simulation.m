%% Simulation Parameters
% --- Array Parameters ---
M = 4;                      
lambda = 0.125;             
R = lambda / (2 * sin(pi/M)); 
k = 2 * pi / lambda;        

% --- Signal Parameters ---
D = 1;                      
theta_source = 60;          
phi_source = 90;            
SNR_dB = -25:1:30;          

% --- Sweep Parameters ---
N_list = [10, 25, 50,];                    % Number of snapshots
q_list = [0.25, 1, 7.5];                 % cos^q antenna directional patterns
trial_list = [25, 50, 100, 250];          % Monte Carlo trial counts

% --- Pre-computation ---
ant_angles = (0:M-1) * (2*pi/M);
ant_pos = R * [cos(ant_angles); sin(ant_angles); zeros(1, M)];

theta_scan = 0:1:180;
phi_scan = 90;

% Results container: RMSE(SNR, N, q, trials)
rmse_results = zeros(length(SNR_dB), length(N_list), length(q_list), length(trial_list));

%% Main Simulation Loop
fprintf('Starting Big DOA Estimation Simulation...\n');
for iN = 1:length(N_list)
    N = N_list(iN);
    for iq = 1:length(q_list)
        q_val = q_list(iq);
        for it = 1:length(trial_list)
            num_trials = trial_list(it);

            fprintf('N=%d | q=%.2f | Trials=%d\n', N, q_val, num_trials);

            for i_snr = 1:length(SNR_dB)
                snr_val = SNR_dB(i_snr);
                sq_err = zeros(1, num_trials);

                for i_trial = 1:num_trials
                    % Generate source and noise
                    s = sqrt(1/2) * (randn(D, N) + 1j * randn(D, N));
                    snr_linear = 10^(snr_val / 10);
                    noise_power = 1 / snr_linear;
                    n = sqrt(noise_power/2) * (randn(M, N) + 1j * randn(M, N));

                    % Array response with cos^q pattern
                    a_dir = create_uca_steering_vector(theta_source, phi_source, k, ant_pos, ...
                                                      'directional', q_val);
                    x = a_dir * s + n;

                    % MUSIC estimation
                    theta_est = music_algorithm(x, D, k, ant_pos, theta_scan, phi_scan, ...
                                                'directional', q_val);

                    sq_err(i_trial) = (theta_est - theta_source)^2;
                end
                rmse_results(i_snr, iN, iq, it) = sqrt(mean(sq_err));
            end
        end
    end
end
fprintf('Simulation finished.\n');

%% Write Results to CSV
csv_filename = 'rmse_results.csv';
fid = fopen(csv_filename, 'w');

% Write header
fprintf(fid, 'SNR_dB,N,q,Trials,RMSE_deg\n');

% Write rows
for iN = 1:length(N_list)
    for iq = 1:length(q_list)
        for it = 1:length(trial_list)
            for i_snr = 1:length(SNR_dB)
                fprintf(fid, '%d,%d,%.2f,%d,%.6f\n', ...
                        SNR_dB(i_snr), N_list(iN), q_list(iq), trial_list(it), ...
                        rmse_results(i_snr, iN, iq, it));
            end
        end
    end
end

fclose(fid);
fprintf('Results written to %s\n', csv_filename);
