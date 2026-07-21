from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np

class ICameraSource(ABC):
    """
    Interface for RGB/Stereo Camera sources.
    """
    @abstractmethod
    def get_frame(self) -> Tuple[bool, np.ndarray, float]:
        """
        Query the next camera frame.
        Returns:
            Tuple[success, image_frame, timestamp_sec]
        """
        pass

    @abstractmethod
    def get_intrinsics(self) -> dict:
        """
        Query camera intrinsic parameters.
        Returns:
            dict containing fx, fy, cx, cy, and dist_coeffs
        """
        pass


class IImuSource(ABC):
    """
    Interface for Inertial Measurement Unit (IMU) sources.
    """
    @abstractmethod
    def get_imu_data(self) -> Tuple[bool, dict, float]:
        """
        Query the latest IMU data.
        Returns:
            Tuple[success, data_dict, timestamp_sec]
            where data_dict has keys:
              - 'accel': [ax, ay, az] in m/s^2
              - 'gyro': [gx, gy, gz] in rad/s
        """
        pass


class IDepthSource(ABC):
    """
    Interface for Depth mapping/stereo depth sources.
    """
    @abstractmethod
    def get_depth_map(self) -> Tuple[bool, np.ndarray, float]:
        """
        Query the latest depth map.
        Returns:
            Tuple[success, depth_map_image, timestamp_sec]
        """
        pass


class ITrackingSource(ABC):
    """
    Interface for raw headset tracking / fused SLAM pose sources.
    """
    @abstractmethod
    def get_pose(self) -> Tuple[bool, np.ndarray, np.ndarray, float]:
        """
        Query the latest tracking pose.
        Returns:
            Tuple[success, position_vector, quaternion_orientation, timestamp_sec]
            where:
              - position_vector: np.array([x, y, z])
              - quaternion_orientation: np.array([w, x, y, z])
        """
        pass
