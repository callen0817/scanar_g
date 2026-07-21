# ScanAR G — Golden Datasets Benchmark Library
This directory contains the permanent golden reference datasets for ScanAR G. 

> [!WARNING]
> Do NOT overwrite these datasets once recorded and verified. They represent the permanent benchmarks for tracking quality, export compatibility, performance monitoring, and regression testing across different software versions.

## Benchmark Environments
1.  **Office_01**: Base closed-loop trajectory. Tests basic translation and orientation drift tracking.
2.  **Office_02**: Repeat office walk with faster dynamic movements and rotations. Tests tracking robustness.
3.  **Hallway_01**: Corridor walk. Tests z-axis drift, forward-translation scaling, and loop-closure recovery in long hallway loops.
4.  **House_01**: Multi-room loop mapping. Tests relocalization, wall bounds projection, and keyplan mapping consistency.
5.  **Outdoor_01**: Pathway walk. Tests outdoor lighting exposure transitions and feature tracking on natural surfaces.
6.  **Warehouse_01**: Large open space mapping. Tests far-field depth estimation constraints and Gaussian density optimization boundaries.
7.  **LowTexture_01**: Feature-sparse room mapping. Serves as the tracking recovery and relocalization failure-limit benchmark.

## Dataset Structure
Each folder must contain:
*   `scene.splat`: The compiled 32-byte WebGL Gaussian splat representation.
*   `scene.ply`: The standard vertex ASCII point cloud.
*   `metadata.json`: Capture parameters and validation output records.
*   `poses/poses.csv` / `poses/poses.tum`: Trajectory mapping coordinates.
*   `imu/imu_raw.csv`: Telemetry logs (if active).
*   `validation_report.json`: Completed verification check parameters.
