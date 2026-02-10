# UAV Tracking System for Azimuth, Elevation, and Range Estimation for Indoor/Outdoor Application
East Texas A&M University - Class of 2026

#  Project Advisors:
Dr. Tayem<br>
Dr. Radaydeh<br>

# Project Team Members:
Logan Boxdorfer<br>
Alden Edwards<br>
Brandon Lewis<br>
Colton Vandeburg<br>
Parker Reeves<br>

# Disclaimer

This write-up is a guide written by the 2026 Senior Design Project Team in order to help future teams understand the contributions done up to the current point. Everything written here demonstrates our group's understanding of the project and steps to success, and all of the information below has attempted to have been proven true in good faith. Please keep this updated and organized as best as possible to achieve this continuous effort. 

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
[Communications Toolbox Support Package for Analog Devices ADALM-Pluto Radio](https://www.mathworks.com/help/supportpkg/plutoradio/index.html)<br>
b. r2024a or newer<br>
[SoC Blockset Support Package for AMD FPGA and SoC Devices](https://www.mathworks.com/matlabcentral/fileexchange/70616-soc-blockset-support-package-for-amd-fpga-and-soc-devices)<br>

# Installation

# Initialization of IIO-Scope

IIO-Scope is picky with how the oscilloscope will let you view receieved signals. You must display signals in powers of 2 (1, 2, 4, 8). Also, since complex signals are being receieved, there is both an I and Q component to each signal, and those are displayed separately.<br>
To view only the I (real) component of signals in IIO-Scope:<br>
Change the color of every other voltage waveform to black.

# Phase Synchronization

[FMComms5 Phase Synchronization](https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms5-ebz/phase-sync)<br><br>
DoA will not be achieved without phase synchronization. The FMComms5 board has built-in capabilities to sync both on-board chips, and subsequently provide phase synchronization.<br><br>
Once IIO-Scope is open, tab over to the settings window, at the top of the screen.<br>






# IPs on Zynq Boards
Colton or Parker need to write this in

# 
