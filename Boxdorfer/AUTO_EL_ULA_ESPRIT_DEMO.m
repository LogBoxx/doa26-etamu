%% AUTO_EL_ULA_ESPRIT_DEMO.m
%% Logan's work (God tier)

clear; clc; close all;

binPath = "C:\msys64\mingw64\bin";
setenv("PATH", binPath + ";" + string(getenv("PATH")));

iioH = "C:\Users\logbo\Documents\iio_matlab.h";
adH  = "C:\Users\logbo\Documents\ad9361_matlab.h";

if ~libisloaded("libiio")
    loadlibrary(fullfile(binPath,"libiio.dll"), iioH);
end
if ~libisloaded("libad9361")
    loadlibrary(fullfile(binPath,"libad9361.dll"), adH);
end

snr_db = 0;
Navg = 10;

source_angles = 0;

algorithm = 'ESPRIT_REF';

num_sources = 1;
dtheta_deg = 0.25;
numtrials = 10000;
num_snapshots = 2^9;
CenterFrequency = 2.4e9;
SamplingRate = 30.72e6;
SamplesPerFrame = 2^14;

max_reconnect = 2;
max_errors = 5;
num_error = 0;

try
    rx_device = adi.FMComms5.Rx('uri','ip:192.168.1.101', ...
        'CenterFrequency', CenterFrequency, ...
        'SamplingRate', SamplingRate, 'SamplesPerFrame', SamplesPerFrame);
    rx_device.EnabledChannels = [1 2 3 4];

    rx_device.SamplingRate         = SamplingRate;
    rx_device.CenterFrequency      = CenterFrequency;
    rx_device.CenterFrequencyChipB = CenterFrequency;

    rx_device.EnableQuadratureTracking        = false;
    rx_device.EnableRFDCTracking              = false;
    rx_device.EnableBasebandDCTracking        = false;
    rx_device.EnableQuadratureTrackingChipB   = false;
    rx_device.EnableRFDCTrackingChipB         = false;
    rx_device.EnableBasebandDCTrackingChipB   = false;

    rx_device.GainControlModeChannel0      = 'slow_attack';
    rx_device.GainControlModeChannel1      = 'slow_attack';
    rx_device.GainControlModeChannel0ChipB = 'slow_attack';
    rx_device.GainControlModeChannel1ChipB = 'slow_attack';

    ddsF     = 1e6;
    ddsScale = 10^(-18/20);

    tx_device = adi.FMComms5.Tx('uri','ip:192.168.1.101', ...
        'CenterFrequency', CenterFrequency, ...
        'SamplingRate', SamplingRate, ...
        'SamplesPerFrame', SamplesPerFrame);

    tx_device.EnabledChannels = [1 2];
    tx_device.DataSource = 'DDS';

    DDSF = zeros(2,2);
    DDSA = zeros(2,2);
    DDSF(:,1) = ddsF;
    DDSA(:,1) = ddsScale;

    tx_device.DDSFrequencies = DDSF;
    tx_device.DDSScales      = DDSA;

    ctx = iio_create_ctx('ip:192.168.1.101');
    ad9361_mcs_sync(ctx);
    ad9361_phase_sync(ctx, int64(CenterFrequency));
    calllib("libiio","iio_context_destroy", ctx);

    rx_device.EnableQuadratureTracking      = false;
    rx_device.EnableQuadratureTrackingChipB = false;

catch ME
    warning('Hardware setup failed. Running in simulation mode.');
    fprintf('Error: %s\n', ME.message);
    simulation_mode = true;
end

if ~exist('simulation_mode', 'var')
    simulation_mode = false;
end

if ~exist('figNum', 'var')
    figNum = 1;
end

el_DOA_deg_m = zeros(numtrials, num_sources);
xaxis_sp_v = linspace(-90, 90, 180/dtheta_deg + 1);


el_hist = nan(Navg, num_sources);

for t = 1:numtrials
    try
        if simulation_mode
            rx_m = generate_simulated_data(4, 1000, source_angles, snr_db);
        else
            rx_m = get_data_fmc5_4elem(rx_device, max_reconnect);
        end

        rx_snapshots = reduce_snapshots(rx_m, num_snapshots);
        
        [el_DOA_deg, sp_dB, xaxis] = esprit_ref_4elem_with_spectrum(rx_snapshots, num_sources, dtheta_deg);

        el_hist = [el_hist(2:end,:); el_DOA_deg(:).'];
        el_avg = mean(el_hist, 1, 'omitnan');

        update_live_plot(algorithm, t, el_DOA_deg, el_avg, xaxis, sp_dB, num_snapshots);

        el_DOA_deg_m(t, :) = el_avg;
        
        fprintf('Trial %d: Estimated DOA = %0.2f deg | Avg(10) = %0.2f deg\n', ...
            t, el_DOA_deg, el_avg);
        
    catch ME
        disp("Error in trial " + t + ": " + ME.message);
        num_error = num_error + 1;
        t = t - 1;
        if num_error >= max_errors
            disp("Exceeded maximum error count. Aborting.");
            rethrow(ME);
        end
    end
end

if ~simulation_mode
    rx_device.release();
    if exist('tx_device','var'), tx_device.release(); end
end

figure(figNum);
est_std = std(el_DOA_deg_m(:));
scatter(el_DOA_deg_m, zeros(size(el_DOA_deg_m)), 'x', 'LineWidth', 1);
title(["Elevation-Only DOA (4-Element ULA)", "Std. = " + num2str(est_std), ...
       "Mean = " + num2str(mean(el_DOA_deg_m(:))) + " deg"]);
xlabel("Elevation (deg)");
ylabel("Azimuth (not used)");
xlim([-90, 90]);
grid on;
figNum = figNum + 1;

function update_live_plot(algorithm, t, el_DOA_deg, el_avg, varargin)
    num_snapshots_arg = varargin{end};

    
    figure(100); clf;
    
    if false
        xaxis_sp_v = varargin{1};
        sp_board1_db_v = varargin{2};
        sp_board2_db_v = varargin{3};
        
        subplot(2,1,1);
        plot(xaxis, sp_dB, 'm', 'LineWidth', 2); hold on;
        xline(el_avg, '--k', 'LineWidth', 2);
        hold off;
        xlabel("Elevation (deg)");
        ylabel("Spatial Spectrum (dB)");
        title("Full-Array Spatial Spectrum");
        xlim([-90, 90]); ylim([-50, 0]);
        grid on;

    else
        xaxis = varargin{1};
        sp_dB = varargin{2};
        
        subplot(2,1,1);
        plot(xaxis, sp_dB, 'm', 'LineWidth', 2);
        xlabel("Elevation (deg)");
        ylabel("Spatial Spectrum (dB)");
        title("Full-Array Spatial Spectrum");
        xlim([-90, 90]); ylim([-50, 0]);
        xline(el_avg);
        grid on;
    end
    
    subplot(2,1,2);
    scatter(el_avg, zeros(size(el_DOA_deg)), 300, 'x', 'LineWidth', 4);
    title("DOA Estimate, Trial: " + num2str(t));
    xlabel("Elevation (deg)");
    xlim([-90, 90]); ylim([-10, 10]); grid on;

    sgtitle("Elevation-Only DOA (" + algorithm + ") - Using " + num2str(num_snapshots_arg) + " snapshots");
    drawnow;
end

function [el_DOA_deg, sp_dB, xaxis] = esprit_ref_4elem_with_spectrum(rx_m, num_sources, dtheta_deg)

    [M,N] = size(rx_m);
    if M ~= 4
        error('Expected rx_m as 4xN for 4-element ULA.');
    end

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
    el_DOA_deg = sort(asind(u));

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

function rx_m = generate_simulated_data(num_antennas, num_samples, source_angles, snr_db)

    d = 0.5;  
    num_sources = length(source_angles);
    
    array_positions = (-(num_antennas-1)/2 : (num_antennas-1)/2)';
    
    rx_m = zeros(num_antennas, num_samples);
    
    source_signals = (randn(num_sources, num_samples) + 1j*randn(num_sources, num_samples))/sqrt(2);
    
    A = zeros(num_antennas, num_sources);
    for i = 1:num_sources
        theta_rad = source_angles(i) * pi/180;
        A(:,i) = exp(1j * 2*pi*d * array_positions * sin(theta_rad));
    end
    
    signal_component = A * source_signals;
    
    signal_power = mean(abs(signal_component(:)).^2);
    noise_power = signal_power / (10^(snr_db/10));
    noise = sqrt(noise_power/2) * (randn(num_antennas, num_samples) + 1j*randn(num_sources, num_samples));
    
    rx_m = signal_component + noise;
end

function [rx_m] = get_data_fmc5_4elem(rx_device, NUM_RETRY)

    valid = false;
    j = 0;
    k = 0;
    
    while ~valid
        try
            raw_data = rx_device();
            rx_m = fliplr(raw_data);

            rx_m = raw_data';
            
            valid = all(size(rx_m) > 0) && ~any(isnan(rx_m(:))) && sum(abs(rx_m(:))) > 0;
        catch ME
            fprintf("Retrying connection...\n");
            pause(1);
            k = k + 1;
            if k >= NUM_RETRY
                fprintf("Failed to connect to device\n");
                rethrow(ME);
            end
        end
        
        if exist('rx_m', 'var') && ~valid
            fprintf("Invalid data...\n");
            j = j + 1;
        end
        
        if (j >= NUM_RETRY) && ~valid
            error("Connecting but not grabbing valid data for some reason");
        end
    end
end

function ctx = iio_create_ctx(uri)
    if isstring(uri)
        uri = char(uri);
    end

    pUri = libpointer('cstring', uri);

    ctx = calllib("libiio","iio_create_context_from_uri", pUri);

    if isempty(ctx) || (isa(ctx,'lib.pointer') && ctx.isNull)
        error("Failed to create IIO context for %s", uri);
    end
end

function ad9361_mcs_sync(ctx)
    ret = calllib("libad9361","ad9361_fmcomms5_multichip_sync", ctx, uint32(0));
    if ret ~= 0
        error("MCS sync failed, ret=%d", ret);
    end
end

function ad9361_phase_sync(ctx, lo_hz)
    ret = calllib("libad9361","ad9361_fmcomms5_phase_sync", ctx, lo_hz);
    if ret ~= 0
        error("Phase sync/cal failed, ret=%d", ret);
    end
end
