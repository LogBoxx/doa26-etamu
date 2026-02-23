% program		DoubleDemo.m
% purpose	    Concurrently estimate azimuth and elevation doa
% usage         Demo script (functions included)
% notes         (1) Azimuth estimation
%               (2) Elevation estimation
% date			2/12/2026
% programmer    2026 Senior Design Project Team

Navg = 10;

algorithm = 'ESPRIT_REF';

num_sources   = 1;
dtheta_deg    = 0.25;
numtrials     = 500;

num_snapshots = 2^11;     % 2048 snapshots (RAW samples, no reduction)

CenterFrequency  = 2.4e9;
SamplingRate     = 30e6;
SamplesPerFrame  = 2^14;

max_reconnect = 2;  % retry attempts

rx_az_device = adi.FMComms5.Rx('uri','ip:192.168.0.101', ...
    'CenterFrequency', CenterFrequency, ...
    'SamplingRate', SamplingRate, ...
    'SamplesPerFrame', SamplesPerFrame);

rx_el_device = adi.FMComms5.Rx('uri','ip:192.168.1.101', ...
    'CenterFrequency', CenterFrequency, ...
    'SamplingRate', SamplingRate, ...
    'SamplesPerFrame', SamplesPerFrame);

rx_az_device.EnabledChannels = [1 2 3 4];
rx_el_device.EnabledChannels = [1 2 3 4];

if ~exist('figNum', 'var')
    figNum = 1;
end

az_DOA_deg_m = zeros(numtrials, num_sources);
el_DOA_deg_m = zeros(numtrials, num_sources);

az_hist = nan(Navg, num_sources);
el_hist = nan(Navg, num_sources);

for t = 1:numtrials

    % ---- grab data ---
    rx_m_az = get_data_fmc5_4elem(rx_az_device, max_reconnect);

    rx_m_el = get_data_fmc5_4elem(rx_el_device, max_reconnect);

    % ---- snapshots ---
    rx_snapshots_az = rx_m_az(:, 1:min(num_snapshots, size(rx_m_az,2)));

    rx_snapshots_el = rx_m_el(:, 1:min(num_snapshots, size(rx_m_el,2)));

    % ---- esprit ----
    [az_DOA_deg, az_sp_dB, az_xaxis] = esprit_ref_4elem_with_spectrum(rx_snapshots_az, num_sources, dtheta_deg);

    [el_DOA_deg, el_sp_dB, el_xaxis] = esprit_ref_4elem_with_spectrum(rx_snapshots_el, num_sources, dtheta_deg);

    % ---- averaging ---
    az_hist = [az_hist(2:end,:); az_DOA_deg(:).'];
    az_avg  = mean(az_hist, 1, 'omitnan');

    el_hist = [el_hist(2:end,:); el_DOA_deg(:).'];
    el_avg  = mean(el_hist, 1, 'omitnan');

    % ---- live plot ---
    update_live_plot(algorithm, t, az_DOA_deg, az_avg, az_xaxis, az_sp_dB, num_snapshots, "Azimuth", 100);

    update_live_plot(algorithm, t, el_DOA_deg, el_avg, el_xaxis, el_sp_dB, num_snapshots, "Elevation", 101);

    az_DOA_deg_m(t, :) = az_avg;
    el_DOA_deg_m(t, :) = el_avg;

    fprintf('Trial %d: AZ Avg(%d) = %0.2f deg || EL Avg(%d) = %0.2f deg\n', ...
        t, az_DOA_deg, Navg, az_avg, el_DOA_deg, Navg, el_avg);

end

figure(figNum);
az_est_std = std(az_DOA_deg_m(:));
scatter(az_DOA_deg_m, zeros(size(az_DOA_deg_m)), 'x', 'LineWidth', 1);
title(["Azimuth DOA (4-Element ULA)", "Std. = " + num2str(az_est_std), ...
       "Mean = " + num2str(mean(az_DOA_deg_m(:))) + " deg"]);
xlabel("Azimuth (deg)");
xlim([-90, 90]);
grid;
figNum = figNum + 1;

figure(figNum);
el_est_std = std(el_DOA_deg_m(:));
scatter(el_DOA_deg_m, zeros(size(el_DOA_deg_m)), 'x', 'LineWidth', 1);
title(["Elevation DOA (4-Element ULA)", "Std. = " + num2str(el_est_std), ...
       "Mean = " + num2str(mean(el_DOA_deg_m(:))) + " deg"]);
xlabel("Elevation (deg)");
xlim([-90, 90]);
grid;
figNum = figNum + 1;

%% ---------------- PLOT FUNCTION ----------------
function update_live_plot(algorithm, t, doa_deg, doa_avg, xaxis, sp_dB, num_snapshots_arg, axis_name, fig_id)

    figure(fig_id); clf;

    subplot(2,1,1);
    plot(xaxis, sp_dB, 'm', 'LineWidth', 2);
    xlabel(axis_name + " (deg)");
    ylabel("Spatial Spectrum (dB)");
    title(axis_name + " Full-Array Spatial Spectrum");
    xlim([-90, 90]); ylim([-50, 0]);
    xline(doa_avg);
    grid on;

    subplot(2,1,2);
    scatter(doa_avg, zeros(size(doa_deg)), 300, 'x', 'LineWidth', 4);
    title(axis_name + " DOA Estimate, Trial: " + num2str(t));
    xlabel(axis_name + " (deg)");
    xlim([-90, 90]); ylim([-10, 10]); grid on;

    sgtitle(axis_name + " DOA (" + algorithm + ") - Using " + num2str(num_snapshots_arg) + " snapshots");
    drawnow;
end

%% ---------------- ESPRIT (CORE UNCHANGED) ----------------
function [az_DOA_deg, sp_dB, xaxis] = esprit_ref_4elem_with_spectrum(rx_m, num_sources, dtheta_deg)

    [M,N] = size(rx_m);

    d = 0.5;

    R = (rx_m*rx_m')/N;

    J = flipud(eye(M));
    R = 0.5 * (R + J*conj(R)*J);

    [U,D] = eig(R);
    [~,idx] = sort(diag(D),'descend');
    U = U(:,idx);

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
    
    xaxis = linspace(-90, 90, 180/dtheta_deg + 1);

    pos = (-(M-1)/2 : (M-1)/2).';

    sp = zeros(size(xaxis));
    for i = 1:numel(xaxis)
        th = xaxis(i)*pi/180;
        a = exp(1j*2*pi*d*pos*sin(th));
        sp(i) = real(a' * R * a);
    end
    sp = sp ./ max(sp + eps);
    sp_dB = 10*log10(sp);
end

%% ---------------- DATA ACQ (keeps your flip) ----------------
function rx_m = get_data_fmc5_4elem(rx_device, max_reconnect)

    for k = 1:max_reconnect
        try
            raw_data = rx_device();      % get 4-ch complex data
            rx_m = fliplr(raw_data);     % keep your flip
            return;
        catch
            if k == max_reconnect
                error("Failed to acquire data after %d attempts.", max_reconnect);
            end
        end
    end
end
