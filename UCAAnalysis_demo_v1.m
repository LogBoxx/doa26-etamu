% ======================================================================= %
% Script:   UCAAnalysis_demo_v1
% Function: Version one performs singal analysis of recieved UCA signals          
% Author:   Terry Cruz
% Date:     11/04/2025
% ======================================================================= %

clear; clf

% =========== FMComms5 INITIALIZATION ========== %

x_direct = adi.FMComms5.Rx('uri','ip:192.168.0.1');
x_direct.EnabledChannels = [1 2 3 4];
x_direct.RFBandwidth = 200e3;
x_direct.CenterFrequency = 2.4e9;

% ============= FFT INITIALIZATION ============= %

Fs = x_direct.SamplingRate;
L = x_direct.SamplesPerFrame;
T = 1/Fs;
t = (0:L-1)*T;

while true
    X = x_direct();
    Y = 10*log(abs(fftshift(fft(X))));
    X_filtered = bandpass(X,[1,1e5],Fs);
    Y_filtered = 10*log(abs(fftshift(fft(X_filtered))));

    subplot(2,2,1)
    plot(Fs/L*(0:L-1)+x_direct.CenterFrequency,Y)
    xlabel('Frequency (Hz)')
    ylabel('Magnitude (dB)')
    title('Raw FFT')

    subplot(2,2,2)
    plot(t,real(X))
    xlim([0,1e-4])
    xlabel('Time (Seconds)')
    ylabel('Magnitude (Some Shit)')
    title('Raw Signal')

    subplot(2,2,3)
    plot(Fs/L*(0:L-1)+x_direct.CenterFrequency,Y_filtered)
    xlabel('Frequency (Hz)')
    ylabel('Magnitude (dB)')
    title('Filtered FFT')

    subplot(2,2,4)
    plot(t,real(X_filtered))
    xlabel('Time (Seconds)')
    ylabel('Magnitude (Dunno)')
    title('Filtered Signal')

    pause(0.2)
end