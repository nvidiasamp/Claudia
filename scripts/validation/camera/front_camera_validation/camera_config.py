#!/usr/bin/env python3
# scripts/validation/camera/front_camera_validation/camera_config.py
# Generated: 2025-06-27
# Purpose: Unitree Go2 front camera configuration and initialization management

import cv2
import time
import json
import logging
import numpy as np
from typing import Dict, Tuple, Optional, Any
from dataclasses import dataclass
import os

@dataclass
class CameraSpec:
    """Camera specification data class"""
    resolution: Tuple[int, int]
    fps: float
    latency_ms: float
    color_format: str
    bit_depth: int

class CameraConfig:
    """Front camera configuration manager"""

    def __init__(self, config_path: str = None):
        """
        Initialize camera configuration

        Args:
            config_path: Configuration file path
        """
        self.logger = logging.getLogger(__name__)
        self.config = self._load_config(config_path)
        self.camera = None
        self.is_initialized = False
        self.actual_spec = None

    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load configuration file"""
        if config_path:
            config_file = config_path
        else:
            # Use path relative to current script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_file = os.path.join(script_dir, "validation_config.json")

        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except FileNotFoundError:
            self.logger.error(f"Configuration file not found: {config_file}")
            return self._get_default_config()
        except json.JSONDecodeError as e:
            self.logger.error(f"Configuration file JSON parse error: {e}")
            return self._get_default_config()

    def _get_default_config(self) -> Dict[str, Any]:
        """Get default configuration"""
        return {
            "camera_config": {
                "target_resolution": [1280, 720],
                "fallback_resolution": [480, 1280],
                "target_fps": 30,
                "camera_id": 0,
                "timeout_seconds": 10
            }
        }

    def initialize_camera(self, method: str = "opencv") -> bool:
        """
        Initialize camera

        Args:
            method: Initialization method ("opencv" or "unitree_sdk")

        Returns:
            bool: Whether initialization was successful
        """
        try:
            if method == "opencv":
                return self._initialize_opencv()
            elif method == "unitree_sdk":
                return self._initialize_unitree_sdk()
            else:
                self.logger.error(f"Unsupported initialization method: {method}")
                return False
        except Exception as e:
            self.logger.error(f"Camera initialization failed: {e}")
            return False

    def _initialize_opencv(self) -> bool:
        """Initialize camera using OpenCV"""
        self.logger.info("Initializing front camera using OpenCV...")

        camera_config = self.config.get("camera_config", {})
        camera_id = camera_config.get("camera_id", 0)

        # Try different backends
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]

        for backend in backends:
            try:
                self.camera = cv2.VideoCapture(camera_id, backend)
                if self.camera.isOpened():
                    self.logger.info(f"Successfully opened camera (ID: {camera_id}, Backend: {backend})")
                    break
                else:
                    self.camera.release()
                    self.camera = None
            except Exception as e:
                self.logger.warning(f"Backend {backend} initialization failed: {e}")
                continue

        if not self.camera or not self.camera.isOpened():
            self.logger.error("All backends failed to open camera")
            return False

        # Configure camera parameters
        if not self._configure_camera_parameters():
            self.logger.error("Camera parameter configuration failed")
            return False

        # Verify configuration
        if not self._verify_camera_configuration():
            self.logger.error("Camera configuration verification failed")
            return False

        self.is_initialized = True
        self.logger.info("OpenCV camera initialization complete")
        return True

    def _initialize_unitree_sdk(self) -> bool:
        """Initialize camera using Unitree SDK"""
        self.logger.info("Initializing front camera using Unitree SDK...")

        try:
            # Unitree SDK camera initialization code should be integrated here
            # Currently using simulated implementation
            self.logger.warning("Unitree SDK integration not yet implemented, falling back to OpenCV")
            return self._initialize_opencv()
        except Exception as e:
            self.logger.error(f"Unitree SDK initialization failed: {e}")
            return False

    def _configure_camera_parameters(self) -> bool:
        """Configure camera parameters"""
        camera_config = self.config.get("camera_config", {})

        # Try to set target resolution
        target_resolution = camera_config.get("target_resolution", [1280, 720])
        fallback_resolution = camera_config.get("fallback_resolution", [480, 1280])
        target_fps = camera_config.get("target_fps", 30)

        # Set resolution
        resolution_set = False
        for resolution in [target_resolution, fallback_resolution]:
            width, height = resolution
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

            # Verify whether resolution was set successfully
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))

            if actual_width == width and actual_height == height:
                self.logger.info(f"Successfully set resolution: {width}x{height}")
                resolution_set = True
                break
            else:
                self.logger.warning(f"Resolution setting failed: target {width}x{height}, actual {actual_width}x{actual_height}")

        if not resolution_set:
            self.logger.warning("Using camera default resolution")

        # Set frame rate
        self.camera.set(cv2.CAP_PROP_FPS, target_fps)
        actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
        self.logger.info(f"Set frame rate: target {target_fps}fps, actual {actual_fps}fps")

        # Set other parameters
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer latency
        self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Auto exposure

        return True

    def _verify_camera_configuration(self) -> bool:
        """Verify camera configuration"""
        try:
            # Test capturing a few frames
            for i in range(5):
                ret, frame = self.camera.read()
                if not ret:
                    self.logger.error(f"Test frame capture failed (frame {i+1})")
                    return False
                time.sleep(0.1)

            # Record actual specifications
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)

            self.actual_spec = CameraSpec(
                resolution=(actual_width, actual_height),
                fps=actual_fps,
                latency_ms=0,  # Will be measured during performance test
                color_format="BGR",
                bit_depth=8
            )

            self.logger.info(f"Camera configuration verified successfully: {actual_width}x{actual_height}@{actual_fps}fps")
            return True

        except Exception as e:
            self.logger.error(f"Camera configuration verification failed: {e}")
            return False

    def capture_frame(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Capture a single frame

        Returns:
            Tuple[bool, Optional[np.ndarray]]: (success flag, image data)
        """
        if not self.is_initialized or not self.camera:
            return False, None

        try:
            ret, frame = self.camera.read()
            return ret, frame
        except Exception as e:
            self.logger.error(f"Image capture failed: {e}")
            return False, None

    def capture_frame_with_timestamp(self) -> Tuple[bool, Optional[np.ndarray], float]:
        """
        Capture a single frame with timestamp

        Returns:
            Tuple[bool, Optional[np.ndarray], float]: (success flag, image data, timestamp)
        """
        timestamp = time.time()
        ret, frame = self.capture_frame()
        return ret, frame, timestamp

    def get_camera_properties(self) -> Dict[str, Any]:
        """Get camera properties"""
        if not self.is_initialized or not self.camera:
            return {}

        properties = {
            "width": int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": self.camera.get(cv2.CAP_PROP_FPS),
            "format": int(self.camera.get(cv2.CAP_PROP_FORMAT)),
            "brightness": self.camera.get(cv2.CAP_PROP_BRIGHTNESS),
            "contrast": self.camera.get(cv2.CAP_PROP_CONTRAST),
            "saturation": self.camera.get(cv2.CAP_PROP_SATURATION),
            "auto_exposure": self.camera.get(cv2.CAP_PROP_AUTO_EXPOSURE),
            "exposure": self.camera.get(cv2.CAP_PROP_EXPOSURE),
            "buffer_size": int(self.camera.get(cv2.CAP_PROP_BUFFERSIZE))
        }

        return properties

    def adjust_camera_settings(self, **kwargs) -> bool:
        """
        Adjust camera settings

        Args:
            **kwargs: Camera property key-value pairs

        Returns:
            bool: Whether adjustment was successful
        """
        if not self.is_initialized or not self.camera:
            return False

        property_map = {
            'brightness': cv2.CAP_PROP_BRIGHTNESS,
            'contrast': cv2.CAP_PROP_CONTRAST,
            'saturation': cv2.CAP_PROP_SATURATION,
            'exposure': cv2.CAP_PROP_EXPOSURE,
            'auto_exposure': cv2.CAP_PROP_AUTO_EXPOSURE,
            'fps': cv2.CAP_PROP_FPS
        }

        for prop_name, prop_value in kwargs.items():
            if prop_name in property_map:
                cv_prop = property_map[prop_name]
                self.camera.set(cv_prop, prop_value)
                self.logger.info(f"Set {prop_name} = {prop_value}")

        return True

    def release(self):
        """Release camera resources"""
        if self.camera:
            self.camera.release()
            self.camera = None
            self.is_initialized = False
            self.logger.info("Camera resources released")

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.release()

# Test function
def test_camera_config():
    """Test camera configuration functionality"""
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    with CameraConfig() as camera_config:
        # Test initialization
        if camera_config.initialize_camera("opencv"):
            print("Camera initialization successful")

            # Test property retrieval
            properties = camera_config.get_camera_properties()
            print(f"Camera properties: {properties}")

            # Test image capture
            ret, frame = camera_config.capture_frame()
            if ret:
                print(f"Image capture successful: {frame.shape}")
            else:
                print("Image capture failed")
        else:
            print("Camera initialization failed")

if __name__ == "__main__":
    test_camera_config()
