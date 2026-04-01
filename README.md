# UAV Tracking System – East Texas A&M Senior Design 2026

Hybrid RF localization system that estimates azimuth, elevation, and range of a moving RF source in real time. Built for the AFRL University Design Challenge.

**Team:** Logan Boxdorfer, Alden Edwards, Brandon Lewis, Colton Vandenburg, Parker Reeves  
**Advisors:** Dr. Tayem, Dr. Radaydeh

---

## What it does

The system tracks a 2.4 GHz RF source by splitting the localization problem into three parts: azimuth estimation, elevation estimation, and range. Azimuth and elevation are handled by two orthogonally mounted antenna arrays feeding into subspace-based DoA algorithms. Range is handled by a LiDAR sensor on a pan-tilt mount that physically points itself toward the estimated direction and measures distance.

The result is a full position estimate in spherical coordinates: azimuth, elevation, and range.

---

## How the signal chain works

A USRP B200mini controlled by a Raspberry Pi transmits a continuous 2.4 GHz tone. The antenna arrays pick it up, the FMComms5 boards digitize it, and MATLAB runs the DoA estimation on the host PC. Once angles are computed they get sent to a second Raspberry Pi that drives the pan-tilt servos and reads back distance from the TF-02 LiDAR.

```
USRP TX (RPi 4) → [RF propagation] → Antenna Array
→ FMComms5 / ZC702 → MATLAB (MUSIC or ESPRIT) → DoA angles
→ RPi pan-tilt controller → TF-02 LiDAR → range
```

---

## Array configurations

Two configurations were tested. The system separates azimuth and elevation into independent 1D estimation problems rather than doing a full 2D search, which cuts computational load significantly.

**UCA / ULA** – circular array handles azimuth, vertical linear array handles elevation. The UCA gives nearly uniform azimuth sensitivity but ESPRIT can't be applied to it directly without extra transforms, so MUSIC is used for the azimuth axis.

**ULA / ULA** – two orthogonal linear arrays, one per axis. Both MUSIC and ESPRIT work cleanly here. This configuration tends to give better subspace separation and more consistent results overall.

---

## MUSIC

MUSIC builds a covariance matrix from multiple snapshots of received data across all antenna channels, then does an eigenvalue decomposition to split the data into a signal subspace and a noise subspace. The key property it exploits is that the true steering vector for the source direction is orthogonal to the noise subspace. MUSIC scans across candidate angles and finds the peak where that orthogonality is strongest.

Forward-backward averaging is applied to the covariance matrix before decomposition. This helps in multipath-heavy environments by decorrelating reflected signals and making the subspace separation more stable.

The UCA needs a modified steering vector compared to the ULA. Because each element sits at a different angular position around the circle, the phase response at each element depends on where it sits relative to the incoming wave direction rather than a simple linear phase progression down the array.

---

## ESPRIT

ESPRIT works by splitting the array into two overlapping subarrays with a known spacing. Because the same wavefront hits both subarrays with a fixed phase offset, the relationship between the two subarray signal subspaces directly encodes the angle of arrival. ESPRIT extracts the DoA from the eigenvalues of that relationship without doing a spectral search at all.

This makes it faster than MUSIC and it tends to need fewer snapshots to stabilize, especially in the ULA/ULA configuration. The downside is it requires the shift-invariant array structure that a UCA doesn't naturally provide.

---

## Simulation results

Monte Carlo simulations were run to test both algorithms under varying SNR and snapshot counts before hardware testing.

For SNR testing, elevation estimation with the vertical ULA was solid across the board. Azimuth with the UCA showed the typical threshold behavior where estimates fall apart at low SNR then snap into accuracy once you cross the threshold. The ULA/ULA configuration improved azimuth stability noticeably for both algorithms.

For snapshot testing, MUSIC with the UCA/ULA setup stabilized around 2048 snapshots. ESPRIT with the ULA/ULA setup was stable even at very low snapshot counts, which makes it more practical for real-time operation where you can't buffer a lot of data.

The main takeaway from simulation is that array geometry has more impact on required snapshot count than algorithm choice. When both axes are ULAs, performance is strong regardless of which algorithm you pick.

---

## Range finding

Range is handled by a TF-02 LiDAR sensor mounted on a servo-driven pan-tilt platform. After DoA estimation produces azimuth and elevation angles, those get sent to a Raspberry Pi that commands the servos to physically point the LiDAR at the estimated source direction. The sensor fires and returns a distance measurement.

Two scripts handle the LiDAR at the root of the repo:

**tf02\_read\_once.m** – connects to the TF-02 and takes a single distance reading. Good for verifying the sensor is working or getting a one-shot range value during setup.

**tf02\_stream.m** – continuously streams distance readings from the TF-02 in real time. This is what actually gets used during live demos when the pan-tilt is actively tracking a moving source.

The rest of the range-finding hardware notes and supporting scripts live in `/Range-Finding/`.

---

## Antenna radiation patterns

The UCA produces a bell-pepper shaped pattern in 3D. In the azimuth plane it has nulls at 0, 90, 180, and 270 degrees. If your source is sitting at one of those nulls the received signal drops and your estimates will suffer.

The ULA has strong response in front of and behind the array but poor response off the sides. Combining a horizontal ULA with a UCA gives complementary coverage in both axes. Running two orthogonal ULAs gives stronger spatial selectivity but introduces more lobes and nulls in the combined pattern.

---

## Live demo hardware

- 2x AMD Zynq ZC702 evaluation boards
- 2x AD-FMCOMMS5-EBZ boards
- 8x 2.4 GHz monopole antennas
- USRP-2901 as the RF transmitter source
- 2x Raspberry Pi 4
- Pan-tilt servo platform
- TF-02 LiDAR sensor
- Ethernet switch or dual NIC host PC

---

## Software setup

### MATLAB version matters

**r2023b or older:**
- [Communications Toolbox Support Package for Xilinx Zynq-Based Radio](https://www.mathworks.com/matlabcentral/fileexchange/48491-communications-toolbox-support-package-for-xilinx-zynq-based-radio)
- [Communications Toolbox Support Package for ADALM-Pluto Radio](https://www.mathworks.com/help/supportpkg/plutoradio/index.html)

**r2024a or newer:**
- [SoC Blockset Support Package for AMD FPGA and SoC Devices](https://www.mathworks.com/matlabcentral/fileexchange/70616-soc-blockset-support-package-for-amd-fpga-and-soc-devices)

### Add-ons
- [RFSoC Explorer Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/73665-rfsoc-explorer-toolbox)
- [ADI Transceiver Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/72645-analog-devices-inc-transceiver-toolbox)
- [Communications Toolbox Support Package for ADALM-Pluto Radio](https://www.mathworks.com/matlabcentral/fileexchange/61624-communications-toolbox-support-package-for-analog-devices-adalm-pluto-radio)
- [MATLAB Support for MinGW-w64 C/C++/Fortran Compiler](https://www.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-fortran-compiler)

Required toolboxes via Add-On Explorer:
- Communications Toolbox
- DSP System Toolbox
- Signal Processing Toolbox
- SoC Blockset

### Linux
Running [ADI Kuiper Linux](https://github.com/analogdevicesinc/adi-kuiper-gen) on both ZC702 boards.

---

## Board IP addresses

Defaults:
- Board A: `192.168.0.1`
- Board B: `192.168.1.1`

Set your host IPs:
```bash
sudo ip addr add 192.168.0.101/24 dev eth0
sudo ip addr add 192.168.1.101/24 dev eth0
```

Run `ip addr` or `ifconfig` on the board to confirm.

---

## Phase sync and calibration

> This is required every time the boards are power cycled. DoA will not work without it.

The FMComms5 boards use Multi-Chip Sync to phase-align all receive channels. Without this the phase relationships between antennas are meaningless and your estimates will be garbage.

1. Wire TX to RX on the board (`TX1_A → RX1_A`, `TX2_A → RX2_A`, `TX1_B → RX1_B`, `TX2_B → RX2_B`)
2. Match all LOs to your target frequency across all four datapaths
3. Disable receiver tracking: Quadrature, RF DC, BB DC
4. Set gain mode to `slow_attack` and adjust manually until RSSI is around 50-55 dB
5. In the AD936X panel go to the FMComms5 tab, hit Reset Calibration, wait ~5 seconds
6. Verify TX phase rotation shows 0, then hit MCS Sync and wait ~5 seconds
7. Hit Calibrate and wait for it to finish
8. Turn off Quadrature tracking again since it tends to re-enable itself
9. Verify TX/RX phase rotation is non-zero after calibration
10. Reconnect to the antenna array

If calibration fails, double check sample rate, RF bandwidth, and LO frequency, and make sure RSSI is above 50 dB. The ADI guides suggest 37-43 dB but that didn't work consistently for us.

Default config we used: 30.72 MSPS sample rate, 18 MHz RF bandwidth, 2.4 GHz LO.

Full details:
- [FMComms5 Phase Sync Wiki](https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms5-ebz/phase-sync)
- [FMComms5 DoA Whitepaper](https://wiki.analog.com/_media/resources/tools-software/linux-software/doa_whitepaper.pdf) — read this before touching the hardware

---

## IIO-Scope note

IIO-Scope expects channel counts in powers of 2. Since signals are complex each channel shows up as I and Q separately. If you only want to see the I component, change every Q channel color to black.

---

## Repo structure

```
/2025 Team/         work from last year's team
/Boxdorfer/         Logan's work
/Edwards/           Alden's work
/Reeves/            Parker's work
/CAD/               antenna array and mount CAD files
/Data Collection/   collected RF and LiDAR data from testing
/Final Report/      final report and docs
/Range-Finding/     LiDAR hardware notes and supporting scripts
tf02_read_once.m    one-shot TF-02 LiDAR read
tf02_stream.m       continuous TF-02 LiDAR stream
```
