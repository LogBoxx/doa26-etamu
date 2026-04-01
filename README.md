# UAV Tracking System – East Texas A&M Senior Design (ClCapstone of 2026)

Hybrid UAV tracking system that estimates **azimuth**, **elevation**, and **range** using Direction-of-Arrival (DoA) from a phased antenna array and LiDAR for range.

**Team:** Logan Boxdorfer, Alden Edwards, Brandon Lewis, Colton Vandenburg, Parker Reeves  
**Advisors:** Dr. Tayem, Dr. Radaydeh

---

## How it works (quick version)

RF signal from the target (UAV beacon/telemetry at 2.4 GHz) hits the antenna array. The FMComms5 boards receive it, the ZC702 handles acquisition, and MATLAB does the DoA processing using MUSIC. LiDAR handles range when it's in the loop.

```
Antenna Array → FMComms5 (AD9361 x2) → ZC702 (Kuiper Linux) → MATLAB → DoA estimates
```

---

## Software setup

### MATLAB version matters here

**r2023b or older:**
- [Communications Toolbox Support Package for Xilinx Zynq-Based Radio](https://www.mathworks.com/matlabcentral/fileexchange/48491-communications-toolbox-support-package-for-xilinx-zynq-based-radio)
- [Communications Toolbox Support Package for ADALM-Pluto Radio](https://www.mathworks.com/help/supportpkg/plutoradio/index.html)

**r2024a or newer:**
- [SoC Blockset Support Package for AMD FPGA and SoC Devices](https://www.mathworks.com/matlabcentral/fileexchange/70616-soc-blockset-support-package-for-amd-fpga-and-soc-devices)

### Toolboxes / Add-ons (install via Add-On Explorer)
- [RFSoC Explorer Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/73665-rfsoc-explorer-toolbox)
- [ADI Transceiver Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/72645-analog-devices-inc-transceiver-toolbox)
- [Communications Toolbox Support Package for ADALM-Pluto Radio](https://www.mathworks.com/matlabcentral/fileexchange/61624-communications-toolbox-support-package-for-analog-devices-adalm-pluto-radio)
- [MATLAB Support for MinGW-w64 C/C++/Fortran Compiler](https://www.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-fortran-compiler)

Required MATLAB toolboxes:
- Communications Toolbox
- DSP System Toolbox
- Signal Processing Toolbox
- SoC Blockset

### Linux
Running [ADI Kuiper Linux](https://github.com/analogdevicesinc/adi-kuiper-gen) on the ZC702 boards.

---

## Board IP addresses

Defaults:
- Board A: `192.168.0.1`
- Board B: `192.168.1.1`

Set your host IPs to match:
```bash
sudo ip addr add 192.168.0.101/24 dev eth0
sudo ip addr add 192.168.1.101/24 dev eth0
```

Use `ip addr` or `ifconfig` on the board to confirm.

---

## Phase sync / calibration

> **This is required every time the boards are power cycled. DoA will not work reliably without it.**

We're using Multi-Chip Sync (MCS) on the FMComms5. High-level steps:

1. Wire TX → RX on the board (`TX1_A → RX1_A`, `TX2_A → RX2_A`, etc.)
2. Match all LOs to your target frequency
3. Disable receiver tracking (Quadrature, RF DC, BB DC) and set gain mode to `slow_attack`
4. Adjust manual gain until RSSI is around **50–55 dB**
5. Run **Reset Calibration → MCS Sync → Calibrate** from the AD936X panel in IIO-Scope
6. Verify TX/RX phase rotation is non-zero after calibration, then reconnect to the array

Full details + background reading:
- [FMComms5 Phase Sync Wiki](https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms5-ebz/phase-sync)
- [DoA + FMComms5 Whitepaper (required reading)](https://wiki.analog.com/_media/resources/tools-software/linux-software/doa_whitepaper.pdf)

---

## Repo structure

```
/2025 Team/         work from last year's team
/Boxdorfer/         Logan's work
/Edwards/           Alden's work
/Reeves/            Parker's work
/CAD/               physical array / mount design files
/Data Collection/   collected RF and LiDAR data
/Final Report/      final report and related docs
/Range-Finding/     LiDAR range-finding scripts and notes
tf02_read_once.m    one-shot TF02 LiDAR read
tf02_stream.m       continuous TF02 LiDAR stream
```

---

## Notes

- IIO-Scope expects channel counts in powers of 2. Since signals are complex, each channel shows I and Q separately — change every Q channel color to black if you only want to see I.
- Default config we used: **30.72 MSPS** sample rate, **18 MHz** RF bandwidth, **2.4 GHz** LO.
- If calibration fails, double-check sample rate, RF bandwidth, LO freq, and that RSSI is above 50 dB.
