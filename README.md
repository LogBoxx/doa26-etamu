# UAV Tracking System for Azimuth, Elevation, and Range Estimation for Indoor/Outdoor Application
East Texas A&M University – Senior Design (Class of 2026)<br>

This repository documents a hybrid tracking system that estimates **azimuth**, **elevation**, and **range** of a UAV (or any RF target source) using:<br>
- **Direction-of-Arrival (DoA)** from a phased antenna array + SDRs (MUSIC-based estimation)<br>
- **Range** from **LiDAR** (when available / integrated)<br><br>

---

# Project Advisors
Dr. Tayem<br>
Dr. Radaydeh<br>

# Project Team Members
Logan Boxdorfer<br>
Alden Edwards<br>
Brandon Lewis<br>
Colton Vandenburg<br>
Parker Reeves<br>

---

# Disclaimer
This guide reflects the 2026 Senior Design team’s understanding at the time of writing. Some steps are hardware/firmware-version dependent (ADI Kuiper, IIO-Scope, MATLAB toolboxes). If anything changes or you find a better approach, **update this README** so the project stays usable.

---

# Problem Statement
UAV tracking (even basic azimuth/elevation + range) is a useful capability for both civil and defense contexts. Many teams can detect RF energy, but **getting stable angle estimates** in real time requires synchronized receivers, correct calibration, and solid signal processing.

---

# Objective Statement
Develop a hybrid DoA + range estimation system using synchronized SDR hardware (FMComms5 / AD9361) with MUSIC-based DoA processing and LiDAR range-finding.

---

# System Overview

## Inputs
- RF signal from target (ex: UAV telemetry / beacon / transmitter at 2.4 GHz)<br>
- Optional LiDAR measurement for range<br>

## Hardware Chain (DoA)
Antenna Array → FMComms5 (AD9361 x2) → ZC702 (Kuiper Linux) → IIO drivers → MATLAB acquisition/processing → DoA estimates<br>

## Outputs
- Estimated **azimuth** and **elevation** angles (and optionally plots / tracking output)<br>
- LiDAR **range** (if connected into the pipeline)<br>

---

# Hardware Required
(2) AMD Zynq™ 7000 SoC **ZC702** Evaluation Kit<br>
(2) **AD-FMCOMMS5-EBZ** (Dual AD9361 Evaluation Board)<br>
(8) Generic **2.4 GHz** monopole omnidirectional antennas<br><br>

Helpful extras
- SMA coax cables + spares<br>
- Power supplies for ZC702 boards<br>
- Ethernet switch or dual NIC host PC<br>
- TX signal source for bench testing (known tone / transmitter / Pluto / signal generator)<br>
- Mounting hardware for consistent array geometry (important)<br>

---

# Software Prerequisites

## ADI / Linux
[ADI Kuiper Linux](https://github.com/analogdevicesinc/adi-kuiper-gen)<br><br>

## MATLAB
Tooling depends on MATLAB version:

### MATLAB r2023b or older
[Communications Toolbox Support Package for Xilinx Zynq-Based Radio](https://www.mathworks.com/matlabcentral/fileexchange/48491-communications-toolbox-support-package-for-xilinx-zynq-based-radio)<br>
[Communications Toolbox Support Package for Analog Devices ADALM-Pluto Radio](https://www.mathworks.com/help/supportpkg/plutoradio/index.html)<br><br>

### MATLAB r2024a or newer
[SoC Blockset Support Package for AMD FPGA and SoC Devices](https://www.mathworks.com/matlabcentral/fileexchange/70616-soc-blockset-support-package-for-amd-fpga-and-soc-devices)<br><br>

## Toolboxes / Add-ons
[RFSoC Explorer Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/73665-rfsoc-explorer-toolbox)<br><br>
[Analog Devices, Inc. Transceiver Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/72645-analog-devices-inc-transceiver-toolbox)<br><br>
[Communications Toolbox Support Package for Analog Devices ADALM-Pluto Radio](https://www.mathworks.com/matlabcentral/fileexchange/61624-communications-toolbox-support-package-for-analog-devices-adalm-pluto-radio)<br><br>
[MATLAB Support for MinGW-w64 C/C++/Fortran Compiler](https://www.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-fortran-compiler)<br><br>

Required MATLAB Toolboxes (install via Add-On Explorer):<br>
- Communications Toolbox<br>
- DSP System Toolbox<br>
- Signal Processing Toolbox<br>
- SoC Blockset<br>

---

# Repository Layout
still needed.

- `/docs/` – papers, references, lab notes, screenshots, diagrams<br>
- `/hardware/` – wiring diagrams, array dimensions, photos, pinouts, settings screenshots<br>
- `/calibration/` – phase sync notes, scripts, verification steps<br>
- `/demo/` – “runs on real hardware” acquisition + DoA demos<br>
- `/simulation/` – offline simulations (array factor, MUSIC/ESPRIT tests, plots)<br>
- `/utils/` – helper functions (covariance, steering vectors, plotting, etc.)<br>

Placeholders currently mentioned:<br>
- `/demo/uca_demo.m`<br>
- `/demo/ula_demo.m`<br>
- `/demo/dual_demo.m`<br>
- `/simulation/2d_plots.m`<br>

---

# Initialization of IIO-Scope
IIO-Scope is picky about display configuration. Signals should be displayed in powers of 2 channels (1, 2, 4, 8). Because the received signals are complex, each channel has an I (real) and Q (imag) component that appear separately.<br><br>

To view only the I (real) component in IIO-Scope:<br>
Change the color of every other waveform (the Q channels) to black.

---

# Phase Synchronization (FMComms5) – REQUIRED

Links:<br>
[FMComms5 Phase Synchronization](https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms5-ebz/phase-sync)<br><br>
[Phase Synchronization Capability of the Analog Devices FMComms5 and DoA Estimation (Whitepaper)](https://wiki.analog.com/_media/resources/tools-software/linux-software/doa_whitepaper.pdf)<br>
**^ REQUIRED READING ^**<br><br>

**DoA estimation will not work reliably without phase synchronization.** This must be done manually **every time the system is power cycled**. The FMComms5 supports Multi-Chip Sync (MCS) and calibration to align phases across receive channels.<br><br>

Before calibrating:<br>
- Close IIO-Scope and reopen it after the board is fully booted.<br>
- Verify default configuration (what we used):<br>
&nbsp;&nbsp;Sample Rate: **30.72 MSPS**<br>
&nbsp;&nbsp;RF Bandwidth: **18.00 MHz**<br>
&nbsp;&nbsp;LOs: **2.4 GHz** by default (adjust as needed)<br><br>

Calibration steps (IIO-Scope):<br>
0. Wire each TX to RX on the FMComms5.<br>
&nbsp;&nbsp;Ensure: **TX1_A -> RX1_A, TX2_A -> RX2_A, TX1_B -> RX1_B, TX2_B -> RX2_B**<br>
1. From the FMComms5 panel, match all LOs to the desired frequency for all four datapaths.<br>
2. Disable receiver tracking: **Quadrature, RF DC, BB DC**<br>
3. Put all receivers into **slow_attack** Gain Control Mode.<br>
4. Set manual receiver gains to get RSSI **50–55 dB**.<br>
&nbsp;&nbsp;(Some guides recommend 37–43 dB, but this didn’t consistently calibrate for us.)<br>
5. From the AD936X panel, select the **FMComms5** tab.<br>
6. Click **Reset Calibration**.<br>
7. Wait ~5 seconds.<br>
8. Verify **TX Phase rotation = 0**.<br>
9. Click **MCS Sync**.<br>
10. Wait ~5 seconds.<br>
11. Click **Calibrate** to launch the procedure.<br>
12. Turn off **Quadrature tracking** again (it may re-enable).<br>
13. Verify **TX/RX Phase rotation ≠ 0** after calibration.<br>
14. Reconnect FMComms5 to the antenna array.<br>
15. Verify stable relative phase behavior across channels (bench test recommended).<br>
16. Optional: try **fast_attack** for better tracking (anecdotal).<br>
17. If **Calibration Failed**: re-check Sample Rate, RF Bandwidth, LO frequency, and ensure RSSI > 50 dB.<br>

---

# Zynq Board IP Addresses
Use `ifconfig` (or `ip addr`) on the board to determine its IP address.<br><br>

Defaults:<br>
A: 192.168.0.1<br>
B: 192.168.1.1<br><br>

IP config (Linux):<br>
`sudo ip addr add 192.168.0.101/24 dev eth0`<br>
`sudo ip addr add 192.168.1.101/24 dev eth0`<br><br>

Run `ifconfig` (or `ip addr`) to verify.

---

# MUSIC
The receiver array measures the same RF signal at multiple antennas. Because each antenna is at a different position, the signal arrives with a **phase difference** that depends on the incoming direction. DoA algorithms use these phase differences to estimate angle.<br><br><br>
- Builds a covariance matrix from multiple snapshots of received data<br>
- Performs eigen-decomposition to separate **signal subspace** and **noise subspace**<br>
- Scans candidate angles and finds peaks where the steering vector is most orthogonal to the noise subspace<br><br>

---

# Scripts
## Demo scrips
`/demo/uca_demo.m` – placeholder<br>
`/demo/ula_demo.m` – placeholder<br>
`/demo/dual_demo.m` – placeholder<br><br>
`/demo/range_demo.m` – placeholder<br><br>
`/demo/all_demo.m` – placeholder<br><br>

## Simus
`/simulation/2d_plots.m` – placeholder<br><br>
`/simulation/2d_plots.m` – placeholder<br><br>
`/simulation/2d_plots.m` – placeholder<br><br>
