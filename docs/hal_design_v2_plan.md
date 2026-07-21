# ScanAR G — V2.1 HAL Design & USB Reverse Engineering Plan
This document defines the architecture of the **Hardware Abstraction Layer (HAL)** implemented for ScanAR G, and outlines the reverse-engineering strategy for decoding raw USB packets from the VITURE Luma Ultra glasses.

---

## 1. HAL Interface Architecture
The HAL decouples the SLAM, mapping, and HUD components from specific physical hardware. All hardware-specific implementations (simulators, local video devices, or custom USB SDK wrappers) must inherit from these abstract base classes defined in `scanar_hal`:

```
                    +--------------------+
                    |    interfaces.py   |
                    +---------+----------+
                              |
       +----------------------+----------------------+
       |                      |                      |
+------+--------+     +-------+-------+     +--------+-------+
| ICameraSource |     |   IImuSource  |     | ITrackingSource|
+------+--------+     +-------+-------+     +--------+-------+
       |                      |                      |
       +----------------------+----------------------+
                              |
                    +---------+----------+
                    |    MockVitureHAL   |
                    +--------------------+
```

### A. ICameraSource
Exposes standard video frame retrieval and intrinsic configurations:
*   `get_frame() -> Tuple[bool, np.ndarray, float]`: Returns `(success, frame_data, timestamp)`.
*   `get_intrinsics() -> dict`: Returns focal length and principal points.

### B. IImuSource
Exposes high-rate inertial measurements:
*   `get_imu_data() -> Tuple[bool, dict, float]`: Returns `(success, {"accel": [x,y,z], "gyro": [x,y,z]}, timestamp)`.

### C. IDepthSource
Exposes active depth map arrays if available:
*   `get_depth_map() -> Tuple[bool, np.ndarray, float]`: Returns `(success, depth_image, timestamp)`.

### D. ITrackingSource
Exposes raw position and quaternion orientation tracked by the headset MCU:
*   `get_pose() -> Tuple[bool, np.ndarray, np.ndarray, float]`: Returns `(success, pos_vector, quat_vector, timestamp)`.

---

## 2. V2.1 USB Protocol Reverse-Engineering Plan
Because the VITURE Luma Ultra glasses register a vendor-specific class (`ID 35ca:1104`) without a standard Linux driver endpoint, the first phase of V2 is dedicated to decoding USB traffic.

### A. Data Capture Strategy
We will capture raw USB descriptors and interrupt packets from the headset endpoints:
1.  **Endpoint 1 IN (Interrupt, 1024-byte packet size)**: Expected main data payload.
2.  **Endpoint 3 IN (Interrupt, 512-byte packet size)**: Secondary event register.

#### Capture Verification Tooling:
We will execute standard USB packet sniffers (like `usbmon` and `wireshark`/`tshark` over a virtual network link) to record the raw hex output while moving the headset.

### B. Payload Decoding Objectives (IMU Discovery)
For each packet captured on EP 1/3, we must isolate:
1.  **Frame Rates**: Track the interval between consecutive packet arrival times (confirming if it streams at 100 Hz, 200 Hz, or on head-movement changes only).
2.  **Data Mappings (Accel & Gyro)**:
    *   Identify 16-bit or 32-bit integer sequences that change linearly when the headset is translated or rotated along individual axes.
    *   Determine byte alignment: **Big-Endian** vs. **Little-Endian**.
    *   Extract scaling factors (converting raw integer counts to $\text{m/s}^2$ and $\text{rad/s}$).
3.  **Timestamp Headers**: Identify 4-byte or 8-byte increasing counters representing the headset's internal hardware clock to ensure synchronization.
4.  **CRC/Checksum**: Identify trailing bytes that validate packet integrity.

---

## 3. V2 Hardware Decision Gate Process
Once the USB payload and SDK outputs are verified, the project will transition to the highest capability SLAM tier supported by the hardware:

```
Case A: RGB + Stereo + IMU Available  --> Tier 1: Full Visual-Inertial Gaussian SLAM
Case B: Stereo + RGB Available        --> Tier 2: Stereo Visual Gaussian SLAM
Case C: RGB + Depth Available         --> Tier 3: RGB-D Gaussian SLAM
Case D: RGB Only                      --> Tier 4: Monocular Gaussian SLAM (V1.5 Base)
```
