# UAV Tracking System – East Texas A&M Senior Design 2026

Hybrid RF localization system that estimates azimuth, elevation, and range of a moving RF source in real time. Built for the AFRL University Design Challenge.

**Team:** Logan Boxdorfer, Alden Edwards, Brandon Lewis, Colton Vandenburg, Parker Reeves  
**Advisors:** Dr. Tayem, Dr. Radaydeh

---
# DOA UAV Tracking System — Project Overview & Operations

**Team:** Logan Boxdorfer, Alden Edwards, Brandon Lewis, Colton Vandenburg, Parker Reeves
**Advisors:** Dr. Tayem, Dr. Radaydeh
**Program:** East Texas A&M — Electrical Engineering Senior Design 2025–2026
**Competition:** AFRL University Design Challenge

---

## What It Is

- A real-time RF localization system that estimates the azimuth, elevation, and range of a moving RF source (simulating a drone/UAV)
- Splits the localization problem into three independent parts: azimuth estimation, elevation estimation, and range
- Uses Direction-of-Arrival (DoA) signal processing algorithms: MUSIC and ESPRIT
- Combines antenna arrays + LiDAR for full 3D spatial localization in spherical coordinates (azimuth, elevation, range)
- Built on software-defined radio hardware; all processing runs in MATLAB

---

## Signal Chain

```
USRP TX (RPi 4) → [RF propagation] → Antenna Array
→ FMComms5 / ZC702 → MATLAB (MUSIC or ESPRIT) → DoA angles
→ RPi pan-tilt controller → TF-02 LiDAR → range
```

- USRP B200-Mini (controlled by Raspberry Pi) transmits a continuous 2.4 GHz tone
- Antenna arrays receive it; FMComms5 boards digitize all 8 channels
- MATLAB on the host PC runs DoA estimation and produces azimuth + elevation angles
- Angles are sent to a second Raspberry Pi that drives the pan-tilt servos
- TF-02 LiDAR physically points at the estimated direction and returns range

---

## Hardware Components

- **FMComms5 / AD9361** — dual RF transceiver boards; handles multi-channel reception and analog-to-digital conversion
- **Xilinx Zynq 7000 SoC (ZC702)** — two boards (Top + Bottom); FPGA + ARM; handles data buffering and transfer to PC
- **8-element antenna array** — hybrid configuration: vertical ULA (elevation) + horizontal ULA or UCA (azimuth)
- **NI USRP B200-Mini / USRP-2901** — the RF transmitter; sits on the RC truck, transmits 2.4 GHz CW signal
- **Raspberry Pi 4 (x2)** — one on RC truck running the transmit script; one controlling the pan-tilt servos
- **Benewake TF-02 LiDAR** — range finder; mounted on pan-tilt servo; steered by DoA estimates
- **RC Kei Truck** — mobile platform carrying the transmitter (USRP + Raspberry Pi + battery)
- **USB-to-Ethernet adapter** — connects PC to both Zynq boards
- **SMA coaxial cables** — length-matched; critical for phase integrity between antenna elements and FMComms5
- **8x 2.4 GHz monopole antennas**
- **Pan-tilt servo platform**
- **Ethernet switch or dual NIC host PC**

---

## Array Configurations

The system separates azimuth and elevation into independent 1D estimation problems rather than doing a full 2D search — cuts computational load significantly.

### UCA / ULA
- Circular array handles azimuth (360° coverage); vertical linear array handles elevation
- UCA gives nearly uniform azimuth sensitivity
- ESPRIT cannot be applied to UCA directly without extra transforms — **MUSIC only** for the azimuth axis on this config

### ULA / ULA
- Two orthogonal linear arrays, one per axis
- Both MUSIC and ESPRIT work cleanly here
- Better subspace separation and more consistent results overall
- More practical for real-time operation due to ESPRIT's speed advantage

---

## Antenna Radiation Patterns

- **UCA** — bell-pepper shaped pattern in 3D; nulls in the azimuth plane at 0°, 90°, 180°, and 270°. If the source is at one of those nulls, received signal drops and estimates will suffer.
- **ULA** — strong response in front of and behind the array but poor response off the sides
- **Horizontal ULA + UCA** — complementary coverage in both axes
- **Two orthogonal ULAs** — stronger spatial selectivity but introduces more lobes and nulls in the combined pattern

---

## Algorithms

### MUSIC

- Builds a covariance matrix from multiple snapshots across all antenna channels
- Eigenvalue decomposition splits data into signal subspace and noise subspace
- Scans candidate angles and finds where the steering vector is most orthogonal to the noise subspace — that's the peak
- Forward-backward averaging applied to covariance matrix before decomposition — decorrelates reflected signals and stabilizes subspace separation in multipath-heavy environments
- UCA requires a modified steering vector vs. ULA — phase response at each element depends on its angular position around the circle, not a simple linear phase progression

### ESPRIT

- Splits the array into two overlapping subarrays with a known spacing
- The same wavefront hits both subarrays with a fixed phase offset — that relationship directly encodes the angle of arrival
- Extracts DoA from eigenvalues — **no spectral search required**
- Faster than MUSIC, needs fewer snapshots to stabilize — more practical for real-time
- **Requires shift-invariant array structure** — UCA doesn't naturally provide this, so ESPRIT is ULA only

---

## Software / Code Structure (MATLAB R2023b)

- `Demo_FMC5_1D_DOA_submission.m` — **main script**; configures hardware, runs acquisition, calls algorithm, plots results
- `sjp_get_doa_MUSIC_QR.m` — MUSIC with QR decomp; best for low snapshots (10–15); primary algorithm
- `sjp_get_doa_OP3R_AZ_8.m` — ESPRIT-like approach for 8-element ULA array
- `sjp_get_doa_RQR_search_az.m` — Rank-Revealing QR; MUSIC-like spectral search method
- `get_data_fmc5_OptB.m` — pulls data from both FMComms5 boards; board 1 = negative side, board 2 = positive side
- `poll_fmc5.m` — low-level hardware comms; handles reconnects and error recovery
- `reduce_snapshots.m` — compresses raw samples into statistical snapshots via segment averaging; boosts SNR, reduces compute load
- `tf02_read_once.m` — single one-shot distance reading from TF-02; use for verifying sensor or setup checks
- `tf02_stream.m` — continuous real-time distance stream from TF-02; what gets used during live demos

---

## Repo Structure

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

---

## MATLAB Setup

### Version Matters

**r2023b or older:**
- [Communications Toolbox Support Package for Xilinx Zynq-Based Radio](https://www.mathworks.com/matlabcentral/fileexchange/48491)
- [Communications Toolbox Support Package for ADALM-Pluto Radio](https://www.mathworks.com/help/supportpkg/plutoradio/index.html)

**r2024a or newer:**
- [SoC Blockset Support Package for AMD FPGA and SoC Devices](https://www.mathworks.com/matlabcentral/fileexchange/70616)

### Required Add-Ons
- [RFSoC Explorer Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/73665)
- [ADI Transceiver Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/72645)
- [Communications Toolbox Support Package for ADALM-Pluto Radio](https://www.mathworks.com/matlabcentral/fileexchange/61624)
- [MATLAB Support for MinGW-w64 C/C++/Fortran Compiler](https://www.mathworks.com/matlabcentral/fileexchange/52848)

### Required Toolboxes (via Add-On Explorer)
- Communications Toolbox
- DSP System Toolbox
- Signal Processing Toolbox
- SoC Blockset

---

## Board OS

Both ZC702 boards run [ADI Kuiper Linux](https://github.com/analogdevicesinc/adi-kuiper-gen).

---

## Network Configuration

| Field | Top Board | Bottom Board |
|---|---|---|
| **AMD Board IP** | 192.168.0.1 | 192.168.1.1 |
| **PC IP** | 192.168.0.11 | 192.168.1.11 |
| **Subnet Mask** | 255.255.255.0 | 255.255.255.0 |
| **Gateway** | Leave blank | Leave blank |
| **DNS** | Leave blank (or 8.8.8.8) | Leave blank (or 8.8.8.8) |

### Set PC IPs (Linux)
```bash
sudo ip addr add 192.168.0.101/24 dev eth0
sudo ip addr add 192.168.1.101/24 dev eth0
```

### Set PC IPs (Windows 10/11)
1. Windows Key + I → Network & Internet → Ethernet
2. Find your Ethernet device → click it
3. Scroll to IP settings → Edit → Manual → IPv4 ON
4. Enter values from table above → Save

### Verify Connection
```bash
ping 192.168.0.1
ping 192.168.1.1
```

### Set AMD Board Static IPs (if not already configured)
```bash
# Open terminal on the board: CTRL + ALT + T
sudo nano /etc/dhcpcd.conf

# Add the correct line for each board:
# Top Board:
static ip_address=192.168.0.1/24
# Bottom Board:
static ip_address=192.168.1.1/24

# Save: Ctrl+O → Enter → Ctrl+X
# Restart DHCP:
sudo systemctl restart dhcpcd
```

Run `ip addr` or `ifconfig` on the board to confirm.

---

## Phase Sync / MCS Calibration

> ⚠️ **Required every time the boards are power cycled. DoA will not work without this.**

The FMComms5 boards use Multi-Chip Sync (MCS) to phase-align all receive channels. Without it, phase relationships between antennas are meaningless and estimates will be garbage.

1. Wire TX to RX on the board: `TX1_A → RX1_A`, `TX2_A → RX2_A`, `TX1_B → RX1_B`, `TX2_B → RX2_B`
2. Match all LOs to your target frequency across all four datapaths
3. Disable receiver tracking: Quadrature, RF DC, BB DC
4. Set gain mode to `slow_attack`; adjust manually until RSSI is around **50–55 dB**
5. In the AD936X panel → FMComms5 tab → hit **Reset Calibration** → wait ~5 seconds
6. Verify TX phase rotation shows 0 → hit **MCS Sync** → wait ~5 seconds
7. Hit **Calibrate** and wait for it to finish
8. Turn off Quadrature tracking again — it tends to re-enable itself
9. Verify TX/RX phase rotation is **non-zero** after calibration
10. Reconnect to the antenna array

**If calibration fails:** check sample rate, RF bandwidth, and LO frequency. Confirm RSSI is above 50 dB. ADI docs suggest 37–43 dB but that was not sufficient — aim for 50–55 dB.

**Default config:** 30.72 MSPS sample rate | 18 MHz RF bandwidth | 2.4 GHz LO

**Reference docs:**
- [FMComms5 Phase Sync Wiki](https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms5-ebz/phase-sync)
- [FMComms5 DoA Whitepaper](https://wiki.analog.com/_media/resources/tools-software/linux-software/doa_whitepaper.pdf) — read this before touching the hardware

---

## IIO-Scope Note

IIO-Scope expects channel counts in powers of 2. Since signals are complex, each channel shows up as I and Q separately. If you only want to see the I component, change every Q channel color to black.

---

## How to Operate — Step by Step

### Step 1: Hardware Setup
- Place 8-element antenna array on stable surface with clear line-of-sight to transmitter track
- Connect antennas to FMComms5 boards via SMA cables
- Connect USB-to-Ethernet adapter from PC → Ethernet cables → each AMD board
- Power on both AMD boards; confirm status LEDs are active

### Step 2: Phase Calibration
- Follow the MCS calibration steps above before doing anything else
- Do this every power cycle — don't skip it

### Step 3: Start the Transmitter
- Load USRP B200-Mini, Raspberry Pi, antenna, and battery onto RC truck
- Power on Raspberry Pi
- Wait ~60–90 seconds — it auto-runs the transmit script; no manual input needed
- USRP starts broadcasting 2.4 GHz signal automatically

### Step 4: Configure Network
- Set static IPs on PC (see Network Configuration above)
- Ping both boards to confirm connection
- If boards aren't responding, check their static IP config via `ip addr` on the board

### Step 5: Run MATLAB
- Launch MATLAB on PC
- Verify all support packages and toolboxes are installed
- Navigate to project folder
- Open and run `Demo_FMC5_1D_DOA_submission.m`
- Script auto-connects to both boards via their IPs

### Step 6: Data Acquisition
- Script acquires synchronized I/Q samples from all 8 antenna elements
- Runs cross-correlation for phase alignment
- Performs snapshot reduction
- Executes MUSIC-QR DoA estimation
- Real-time plots update live as source moves

---

## Expected Outputs

- **Spatial spectrum plot** — signal power vs. angle; peaks = transmitter direction
- **Real-time azimuth angle estimate** — updates continuously as source moves
- **Stationary source accuracy** — within ±2° of true angle
- **Moving source** — smooth tracking with minor jitter depending on speed and multipath
- **Optional batch mode** — mean estimates, standard deviations, trajectory comparison plots

---

## Performance Specs

| Parameter | Value |
|---|---|
| Operating Frequency | 2.4 GHz |
| UCA Azimuth Coverage | 360° |
| UCA Azimuth Accuracy | ±5° |
| Horizontal ULA Azimuth Coverage | 180° |
| Horizontal ULA Azimuth Accuracy | ±3° |
| Vertical ULA Elevation Coverage | 90° |
| Vertical ULA Elevation Accuracy | ±3° |
| DoA Estimation Latency | ~100ms |
| LiDAR Range Finding Latency | 10ms |
| Max Elements per FMComms5 Board | 4 |
| UCA Algorithm Support | MUSIC only |

---

## Simulation Results

Monte Carlo simulations tested both algorithms under varying SNR and snapshot counts before hardware testing.

- **SNR testing** — elevation with the vertical ULA was solid across the board. UCA azimuth showed typical threshold behavior — estimates fall apart at low SNR then snap into accuracy past the threshold. ULA/ULA improved azimuth stability for both algorithms.
- **Snapshot testing** — MUSIC with UCA/ULA stabilized around 2048 snapshots. ESPRIT with ULA/ULA was stable at very low snapshot counts — more practical for real-time.
- **Key takeaway** — array geometry has more impact on required snapshot count than algorithm choice. When both axes are ULAs, performance is strong regardless of algorithm.

---

## Known Limitations / Gotchas

- Multipath interference will always affect results — minimize reflective surfaces during testing
- FMComms5 boards cannot be externally synced — limits array to 4 elements per board
- Phase calibration (MCS) **must** be done every power cycle — DoA will not work without it
- UCA cannot use ESPRIT — MUSIC only
- UCA has nulls at 0°, 90°, 180°, 270° — avoid placing the source at those angles during demos
- RSSI must be 50–55 dB for reliable calibration — ADI's suggested 37–43 dB was not sufficient
- Battery packs must be fully charged before mobile tests
- Limited budget/timeline constrained optimization

---

## Drive Cloning (AMD Board Backup)

> ⚠️ **Warning:** Double-check source vs. target drives — mixing them up causes irreversible data loss.

```bash
# Identify drives
lsblk

# Unmount target
sudo umount /dev/sdb1

# Clone
sudo dd if=/dev/sda of=/dev/sdb bs=4M status=progress

# Sync
sync

# Power off target safely
udisksctl power-off -b /dev/sdb
```

### Wipe a Drive (Zero Fill)
```bash
sudo dd if=/dev/zero of=/dev/sdb bs=4M status=progress
sync
```
