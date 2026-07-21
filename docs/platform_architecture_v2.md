# ScanAR G — V2 Platform Architecture & Refactoring Plan
This document defines the frozen application framework boundaries, details the newly extended Hardware Abstraction Layer (HAL) interfaces, and maps the long-term directory structure of the ScanAR G codebase as it transitions from a standalone application into a robust platform.

---

## 1. Frozen Application Framework
To prevent scope creep and preserve stability, the following elements of ScanAR G are declared **Feature Frozen**:
*   **GUI & HUD Layer** (frozen, all visualization widgets, diagnostic overlays, keyplan floor plan canvas).
*   **Session Management & Exporters** (frozen, directory generator, validation reports, metadata writers).
*   **Lite Preview & Dataset Schemas** (frozen, TUM/CSV trajectory output, WebGL 32-byte splats).

All future improvements to sensing, mapping accuracy, tracking solvers, or hardware drivers **must** be implemented behind the HAL, preserving these frozen components unchanged.

---

## 2. Platform Repository Structure

The codebase is refactored to align with the following modular platform directory structure:

```text
scanar_g/
├── apps/
│   └── scanar_g              # User application (frozen GUI, HUD, and bringup)
├── core/
│   ├── scanar_hal            # Hardware Abstraction Layer interfaces (Python/C++)
│   ├── scanar_tracking       # SLAM frontend, EKF state solvers, and VIO
│   ├── scanar_mapping        # Gaussian point database, densification, and pruning
│   ├── scanar_renderer       # 3D visualization layers
│   └── scanar_export         # Standard 32-byte splat and PLY generators
├── drivers/
│   ├── viture_sdk            # Official SDK mapping layer (OpenXR / Linux SDK)
│   ├── viture_protocol       # Custom vendor-specific USB endpoint parser
│   └── mock_driver           # Dynamic high-fidelity simulated hardware driver
├── tools/
│   ├── usb_decoder           # Raw packet captures, logging, and hex dump analyzers
│   ├── dataset_validator     # Binary alignment and structure validation tool
│   └── benchmarks            # Drift analysis, frame rate, and resource profiling
├── tests/                    # Integration and regression test suites
└── docs/                     # Design plans and stabilization reports
```

---

## 3. Extended HAL Interfaces

### A. ITimeSource
Centralizes high-precision timestamping and clock queries to guarantee timestamp consistency and drift detection:
*   `camera() -> float`: Returns the hardware/driver timestamp (in seconds) of the latest camera exposure.
*   `imu() -> float`: Returns the hardware/driver timestamp (in seconds) of the latest IMU measurement.
*   `pose() -> float`: Returns the tracking update timestamp (in seconds).
*   `system() -> float`: Returns the monotonic reference system clock (in seconds).

### B. IRecorder
Decouples capture logging from the individual driver implementations, providing a uniform recording mechanism for dataset creation:
*   `start_recording(destination_directory: str) -> bool`: Begins recording raw sensor telemetry to the target path.
*   `stop_recording() -> bool`: Stops active telemetry recording and flushes output buffers.
*   `is_recording() -> bool`: Returns `True` if a recording session is active.

---

## 4. Engineering Tracks for V2 Development
The development team is organized into three parallel, decoupled tracks:

```
+---------------------------------------+
|             TRACK A: HARDWARE         |
|  (Raw USB -> Packets -> Decoders ->   |
|         Exposing VitureHAL)           |
+-------------------+-------------------+
                    |
                    v (Implements)
+-------------------+-------------------+
|               CORE HAL                |
|      (ICameraSource, IImuSource,      |
|       ITimeSource, IRecorder)         |
+-------------------+-------------------+
                    |
                    v (Consumed by)
+-------------------+-------------------+
|             TRACK B: TRACKING         |
|  (State Estimators, EKF correction,   |
|     Visual Odometry, Loop Closure)    |
+-------------------+-------------------+
                    |
                    v (Verified by)
+-------------------+-------------------+
|            TRACK C: VALIDATION        |
|    (Drift benchmarks, FPS tracking,   |
|     resource logs, validation reports)|
+-------------------+-------------------+
```
