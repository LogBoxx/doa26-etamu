# UAV Tracking System for Azimuth, Elevation, and Range Estimation for Indoor/Outdoor Application
East Texas A&M University - Class of 2026

#  Project Advisors:
Dr. Tayem<br>
Dr. Radaydeh<br>

# Project Team Members:
Logan Boxdorfer<br>
Alden Edwards<br>
Brandon Lewis<br>
Colton Vandenburg<br>
Parker Reeves<br>

# Disclaimer

This write-up is a guide written by the 2026 Senior Design Project Team in order to help future teams understand the contributions done up to the current point. Everything written here demonstrates our group's understanding of the project and steps to success, and all of the information below has been believed to be true. Please keep this updated and organized as best as possible to achieve this continuous effort. 

# Problem Statement:
Creating effective means of tracking Unmanned Aerial Vehicles (UAVs) has become a prevalent objective in warfare and civil environments

# Objective Statement: 
Develop a hybrid Direction-of-Arrival (DoA) system with synchronized SDR hardware utilizing MUSIC algorithms and LiDAR range-finding 

# Deliverables:
Fully implemented hybrid DoA and range estimation system capable of real-time UAV tracking in azimuth/elevation planes<br>
Publication of experimental results and presentation in the AFRL University Design Challenge

# Required:
(2) AMD Zynq™ 7000 SoC ZC702 Evaluation Kit<br>
(2) AD-FMCOMMS5-EBZ (Dual AD9361 Evaluation Board)<br>
(8) generic 2.4GHz monopole omnidirectional antenna<br>

# Prerequisites
[ADI Kuiper Linux](https://github.com/analogdevicesinc/adi-kuiper-gen)<br><br>
MATLAB<br>
a. r2023b or older<br>
[Communications Toolbox Support Package for Xilinx Zynq-Based Radio](https://www.mathworks.com/matlabcentral/fileexchange/48491-communications-toolbox-support-package-for-xilinx-zynq-based-radio)<br>
[Communications Toolbox Support Package for Analog Devices ADALM-Pluto Radio](https://www.mathworks.com/help/supportpkg/plutoradio/index.html)<br><br>
b. r2024a or newer<br>
[SoC Blockset Support Package for AMD FPGA and SoC Devices](https://www.mathworks.com/matlabcentral/fileexchange/70616-soc-blockset-support-package-for-amd-fpga-and-soc-devices)<br><br>
[RFSoC Explorer Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/73665-rfsoc-explorer-toolbox)<br><br>
[Analog Devices, Inc. Transceiver Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/72645-analog-devices-inc-transceiver-toolbox)<br><br>
[Communications Toolbox Support Package for Analog Devices ADALM-Pluto Radio](https://www.mathworks.com/matlabcentral/fileexchange/61624-communications-toolbox-support-package-for-analog-devices-adalm-pluto-radio)<br><br>
[MATLAB Support for MinGW-w64 C/C++/Fortran Compiler](https://www.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-fortran-compiler)


Communications Toolbox<br>
DSP System Tooblox<br>
Signal Processing Tooblox<br>
SoC Blockset<br>






# Initialization of IIO-Scope

IIO-Scope is picky with how the oscilloscope will let you view receieved signals. You must display signals in powers of 2 (1, 2, 4, 8). Also, since complex signals are being receieved, there is both an I and Q component to each signal, and those are displayed separately.<br>
To view only the I (real) component of signals in IIO-Scope:<br>
Change the color of every other voltage waveform to black.

# Phase Synchronization

[FMComms5 Phase Synchronization](https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms5-ebz/phase-sync)<br><br>
[Phase Synchronization Capability of the
Analog Devices FMComms5 and DoA Estimation](https://wiki.analog.com/_media/resources/tools-software/linux-software/doa_whitepaper.pdf)<br> ^ REQUIRED READING ^ <br><br>
DoA will not be achieved without phase synchronization. This step must be done manually every time the system is power cycled. The FMComms5 board has built-in capabilities to sync both on-board chips, and subsequently provide phase synchronization.<br><br>
Close IIO-Scope and reopen it once the board it booted. To verify the default configuration, the sample rate should be at 30.72 MSPS with RF Bandwidth 18.00 MHz. All LOs should be at 2.4 GHz by default.<br>
To calibrate FMComms5, perform the following within IIO-Scope:<br>
0. Wire each TX to RX on FMComms5. ____Ensure TX1_A -> RX1_a, TX2_A -> RX2_A, TX1_B -> RX1_B, TX2_B -> RX2_B____
1. From the FMComms5 panel, match all of the LOs to the desired frequency for all four datapaths.<br>
2. Disable all receiver trackings (right hand side of page): Quadrature, RF DC, BB DC<br>
3. Put all receivers into **slow_attack** Gain Control Mode
4. Set manually hardware gains for all receivers to achieve an RSSI of 50-55dB (The guide says 37-43, however this does not achieve calibration for our group).<br>
5. From the AD936X panel, select FMComms5 tab.
6. Click _Reset Calibration_
7. Wait 5s
8. Verify TX Phase rotation = 0
9. Click _MCS Sync_ at the bottom
10. Wait 5s
11. Click _Calibrate_, which will launch the procedure
12. Turn off Quadrature tracking again
13. Verify TX, RX Phase rotation ≠ 0
14. Once verified, wire FMComms5 to antenna array
15. To verify phase synchronization, tab back to oscilloscope view and view<br>
16. Consider changing to _fast_attack_ for better estimation (anecdotal-not researched
17. If _Calibration Failed_, ensure correct Sample Rate, RF Bandwidth, LO frequency, and RSSI > 50dB 

# IPs on Zynq Boards<br>
Use _ifconfig_ in shell to determine local IP address<br><br>
Defaults:<br>
A: 192.168.0.1<br>
B: 192.168.1.1<br>

# Script Usage

DoubleDemo.m



