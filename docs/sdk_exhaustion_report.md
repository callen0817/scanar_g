# ScanAR G — SDK Exhaustion & Sensor Capability Report
This document compiles the exhaustive investigation of the official VITURE SDK, OpenXR runtime, and Linux driver APIs for the VITURE Luma Ultra glasses on the Jetson Orin NX platform, establishing the final sensor capability matrix.

---

## 1. SDK Capability Matrix

| Sensor / Capability | Linux SDK | Android SDK | OpenXR | Accessible on Jetson? | Notes / Verification Method |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **RGB Camera** | N/A | Exposed | N/A | **Yes** | Accessible via standard V4L2 device node `/dev/video0`. Verified at $1920 \times 1080$ resolution. |
| **Stereo Left** | N/A | Restricted | N/A | **No** | Closed by firmware. Not enumerated on USB UVC descriptors. |
| **Stereo Right** | N/A | Restricted | N/A | **No** | Closed by firmware. Not enumerated on USB UVC descriptors. |
| **Hardware Depth** | N/A | N/A | N/A | **No** | No active depth/IR hardware exists on this glasses version. |
| **IMU Raw (Accel/Gyro)**| Restricted | Exposed | N/A | **No** | Requires custom USB serial packet decoding because no official Linux SDK binary or OpenXR runtime is compiled for Jetson. |
| **Head Pose (3-DOF)** | Restricted | Exposed | Supported | **No** | Calculated on-board, but orientation registers require custom serial packet decoding over vendor-specific USB interface `35ca:1104`. |
| **Camera Intrinsics** | N/A | Exposed | N/A | **No** | Not exposed via standard UVC query interfaces. Must use checkerboard calibration. |
| **Lens Distortion** | N/A | Pre-applied | N/A | **Yes** | Pre-compensated by internal optical prism calibration. |
| **Exposure / Gain** | N/A | Auto-only | N/A | **Yes** | Controlled internally by the camera hardware auto-exposure loop. |
| **Timestamp** | N/A | Software | N/A | **Software** | System timestamps are assigned by the Linux kernel upon packet arrival (no hardware clock sync sync header). |

---

## 2. Platform Architecture Implications
Based on the results of the SDK exhaustion investigation, the **ScanAR G** product profile is locked to the **monocular camera configuration**:

```
                  +----------------------------------+
                  |         VITURE Glasses           |
                  |     (Only /dev/video0 UVC)       |
                  +----------------+-----------------+
                                   |
                                   v
                  +----------------------------------+
                  |      LingBot-Map Backend         |
                  |  (Monocular Visual Odometry)     |
                  +----------------+-----------------+
                                   |
                                   v
                  +----------------------------------+
                  |         ScanAR Viewer            |
                  |  (Dense 3D RGB Reconstruction)   |
                  +----------------------------------+
```

### Transition of Reconstruction Backends
1.  **ScanAR G (VITURE)**: Locks to **LingBot-Map** monocular camera tracking. Floating 3D Gaussian bubble simulations are removed from the viewer.
2.  **ScanAR S / S2 (OAK-D)**: Exclusively uses the **VIGS-SLAM** backend to process stereo visual-inertial datasets.
3.  **ScanAR L / L2 (Airy LiDAR)**: Employs the **FAST-LIO2** point-cloud mapper.
4.  **ScanAR Pro (Airy + OAK-D)**: Employs the **FAST-LIVO2** dense visual + LiDAR backend.
