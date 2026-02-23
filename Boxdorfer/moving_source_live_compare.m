%% moving_source_live_compare.m

clear; close all; clc;

%% -------- Settings --------
d = 0.5;                    % spacing (wavelengths)
arrayIdx = (-2:1).';         % 4-element ULA indices

N = 250;                     % snapshots per frame
SNR_dB = -12;                  % improvements from -15 and up
num_sources = 1;
dtheta_deg = 0.25;           % scan grid
xaxis = -90:dtheta_deg:90;

% Motion profile (slow sweep)
theta_start = -60;
theta_end   =  60;
nFrames     = 180;
theta_true_vec = linspace(theta_start, theta_end, nFrames);

% Multipath (coherent BETTERlection)
BETTERl_offset_deg = 45;        % BETTER reflection angle = direct + offset
alpha_mag = 0.76;             % BETTER reflection strength
alpha_phase_deg = 25;        % phase of BETTERlection
alpha = alpha_mag * exp(1j*alpha_phase_deg*pi/180);

% Timing
pause_s = 0.0001;          

rng(1);

%% -------- Logging for MSE --------
true_log = nan(nFrames,1);
A_log    = nan(nFrames,1);
B_log    = nan(nFrames,1);

%% -------- Figure setup --------
hFig = figure('Name','Moving source DOA: Old ESPRIT vs New ESPRIT (multipath)','Color','w');
hAx  = axes('Parent',hFig);
hLine = plot(hAx, xaxis, nan(size(xaxis)),'LineWidth',1.5);
grid(hAx,'on');
xlabel(hAx,'Azimuth (deg)');
ylabel(hAx,'Bartlett (dB)');
title(hAx,'Live Bartlett spectrum with DOA overlays');

% Vertical markers (create once, update each frame)
%hTrue  = xline(hAx, 0, '--', 'True direct','LineWidth',1.2);
%hBETTERl  = xline(hAx, 0, '--', 'BETTERlection','LineWidth',1.2);
hAlgA  = xline(hAx, 0, ':',  'Old ESPRIT','LineWidth',1.6);
hAlgB  = xline(hAx, 0, ':',  'New ESPRIT','LineWidth',1.6);

% Text readout
hTxt = text(hAx, -88, -0.5, '', 'FontName','Consolas','FontSize',10);

%% -------- Main loop --------
for t = 1:nFrames
    if ~isvalid(hFig), break; end

    theta_direct_deg = theta_true_vec(t);
    theta_BETTERl_deg   = theta_direct_deg + BETTERl_offset_deg;

    % Steering vectors
    th1 = theta_direct_deg*pi/180;
    th2 = theta_BETTERl_deg*pi/180;
    a1 = exp(1j*2*pi*d*arrayIdx*sin(th1));
    a2 = exp(1j*2*pi*d*arrayIdx*sin(th2));

    % Source (coherent multipath uses same s)
    s = (randn(1,N) + 1j*randn(1,N))/sqrt(2);

    s2 = filter([1 0.2], 1, (randn(1,N)+1j*randn(1,N))/sqrt(2));

    % Noise for target SNR
    signal_power = mean(abs(s).^2);
    noise_power  = signal_power / (10^(SNR_dB/10));
    n = sqrt(noise_power/2) * (randn(4,N) + 1j*randn(4,N));

    % Received data: direct + coherent BETTERlection + noise
    % CHOOSE ONE
    %rx_m = a1*s + alpha*a2*s + n;
    rx_m = a1*s + alpha*a2*s2 + n;

    % Old ESPRIT (your function)
    try
        azA_deg = esprit_4elem(rx_m, num_sources, dtheta_deg);
        if iscell(azA_deg) % just in case user returns weird format
            azA_deg = azA_deg{1};
        end
        if numel(azA_deg) > 1, azA_plot = azA_deg(1); else, azA_plot = azA_deg; end
    catch
        azA_plot = NaN;
    end

    % New ESPRIT (BETTER ESPRIT)
    try
        azB_deg = esprit_BETTER_4elem(rx_m, num_sources, d);
        if numel(azB_deg) > 1, azB_plot = azB_deg(1); else, azB_plot = azB_deg; end
    catch
        azB_plot = NaN;
    end

    % Log estimates for MSE
    true_log(t) = theta_direct_deg;  % compare against DIRECT path truth
    A_log(t)    = azA_plot;
    B_log(t)    = azB_plot;

    
    % Bartlett spectrum
    R = (rx_m*rx_m')/N;
    bart = zeros(size(xaxis));
    for i=1:numel(xaxis)
        th = xaxis(i)*pi/180;
        afull = exp(1j*2*pi*d*arrayIdx*sin(th));
        bart(i) = real(afull' * R * afull);
    end
    bart_db = 10*log10(bart / max(bart + eps)); % normalize safely

    % Update plot
    set(hLine, 'YData', bart_db);
    set(hAx, 'YLim', [min(bart_db)-0.1, 0.1]);

    hAlgA.Value = azA_plot;
    hAlgB.Value = azB_plot;

    set(hTxt,'String',sprintf([ ...
        'Frame %3d/%3d | SNR=%g dB | N=%d\n' ...
        'True = %+6.2f deg | BETTERl = %+6.2f deg (|alpha|=%.2f, phase=%g deg)\n' ...
        'Old ESPRIT = %+6.2f deg | New ESPRIT = %+6.2f deg'], ...
        t, nFrames, SNR_dB, N, theta_direct_deg, theta_BETTERl_deg, alpha_mag, alpha_phase_deg, ...
        azA_plot, azB_plot));

    drawnow;
    pause(pause_s);
end

%% -------- MSE summary over the whole run --------
validA = ~isnan(A_log);
validB = ~isnan(B_log);

errA = wrap180(A_log(validA) - true_log(validA));
errB = wrap180(B_log(validB) - true_log(validB));

mseA = mean(errA.^2);
mseB = mean(errB.^2);

fprintf('\n===== DOA MSE vs TRUE DIRECT over run =====\n');
fprintf('Old ESPRIT: MSE = %.4f deg^2 (RMSE = %.4f deg), valid %d/%d frames\n', ...
    mseA, sqrt(mseA), sum(validA), nFrames);
fprintf('New ESPRIT: MSE = %.4f deg^2 (RMSE = %.4f deg), valid %d/%d frames\n', ...
    mseB, sqrt(mseB), sum(validB), nFrames);

figure('Color','w'); 
plot(true_log, errA, '.-'); hold on; grid on;
plot(true_log, errB, '.-', 'linewidth', 1.2);
xlabel('True DOA (deg)'); ylabel('Error (deg)');
title('Per-frame DOA error vs true direct');
legend('Old ESPRIT error','New ESPRIT error','Location','best');


%% ===== BETTER ESPRIT (proper overlapping subarrays) =====
function az_DOA_deg = esprit_BETTER_4elem(rx_m, num_sources, d)
    [M,N] = size(rx_m);
    if M ~= 4, error('Expected rx_m as 4xN'); end

    R = (rx_m*rx_m')/N;

    [U,D] = eig(R);
    [~,idx] = sort(diag(D),'descend');
    U = U(:,idx);

    if num_sources < 1 || num_sources > 3
        error('num_sources must be 1..3 for 4-element ESPRIT');
    end
    Es = U(:,1:num_sources);

    J1 = [eye(3), zeros(3,1)];
    J2 = [zeros(3,1), eye(3)];

    E1 = J1*Es;
    E2 = J2*Es;

    Psi = pinv(E1)*E2;
    ev = eig(Psi);

    u = angle(ev)/(2*pi*d);
    u = max(min(real(u),1),-1);
    az_DOA_deg = sort(asind(u));
end

function y = wrap180(x)
    % Wrap degrees to [-180, 180)
    y = mod(x + 180, 360) - 180;
end
