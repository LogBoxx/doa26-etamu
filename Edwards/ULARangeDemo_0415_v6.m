% Script    ULADemo_0302_v4
% Purpose  	V4 Implements FB averaging
% Notes     (1) Collects data for dynamic testing
% Author    Parker Reeves
% Date     	03/02/2026

clear; clf
u = udpport("IPV4", "LocalPort", 5005);
rpi = raspi('169.254.67.1', 'analog', 'analog');

if ~exist('s_az', 'var')
    s_az = servo(rpi, 13, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
end
if ~exist('s_el', 'var')
    s_el = servo(rpi, 12, 'MinPulseDuration', 5.44e-4, 'MaxPulseDuration', 2.40e-3);
end
point_servo('reset');
x_direct = adi.FMComms5.Rx('uri','ip:192.168.1.101');
x_direct.EnabledChannels = [1 2 3 4];

x_direct.SamplesPerFrame = 2^15;

M = 4;                   % Number of array elements
d = 0.5;                 % Element spacing (in wavelengths)
c = 3e8; f = 2.4e9;      % Speed of light and operating frequency
lambda = c/f;            % Wavelength (m)
k = 2*pi/lambda;         % Wave number (rad/m)
delta = d*lambda;        % Element spacing in meters

theta_scan = -90:0.5:90;                % Angle search grid (degrees)

num_trials = 100;

m_AZ = zeros(1,num_trials);

Navg = 5; %LOGAN
est_hist = nan(Navg,1); %LOGAN

for o = 1:num_trials
    X = x_direct();                         % Noisy received signal matrix
    J = fliplr(eye(M));

    En = MusicAlg_v2(X,J);
    
    Pmusic = zeros(size(theta_scan));       % Initialize MUSIC spectrum
    

    for t = 1:length(theta_scan)
        a_scan = exp(-1j*k*(0:M-1)'*delta*sind(theta_scan(t)));              % Steering vector for scan angle
        Pmusic(t) = 1 / abs(a_scan'*(En*En')*a_scan);                        % MUSIC pseudo-spectrum value !!!REAL/ABS!!!
    end
    
    [~, idx_peak] = max(Pmusic);            % Find peak index in MUSIC spectrum
    est_DOAs_x = theta_scan(idx_peak);         % Store estimated DOA for this trial

    est_hist = [est_hist(2:end,:); est_DOAs_x(:).']; %LOGAN
    est_avg  = mean(est_hist, 1, 'omitnan'); %LOGAN
    
    [cmd_az, cmd_el, status] = point_servo_v2(rpi, est_avg, 0, s_az, s_el);
    fprintf('Estimated DOA: %.2f°\n', est_avg);
    dist_m = read_range(u,4);
    %pause(0.1)

    m_AZ(o) = est_avg;

end

subplot(2,1,1)
plot(m_AZ,'rx','MarkerSize',8,'LineWidth',1.5)
xlim([0,num_trials])
ylim([-90,90])
title('ULA SVD-QR MUSIC')
xlabel('Trial Number')
ylabel('Azimuth Estimation')

subplot(2,1,2)
plot(dist_m,'bx','MarkerSize',8,'LineWidth',1.5)
xlim([0,num_trials])
ylim([0,10])
title('Distance')
xlabel('Trial Number')
ylabel('Distance(m)')