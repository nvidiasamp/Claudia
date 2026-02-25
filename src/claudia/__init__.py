# src/claudia/__init__.py
"""
Claudia Intelligent Quadruped Robot System

An intelligent quadruped robot system based on the Unitree Go2 R&D Plus platform,
featuring advanced natural language interaction through deep integration of
large language model technology.

Main functional modules:
- robot_controller: Robot control and action execution
- ai_components: AI components (LLM, ASR, TTS, wake word detection)
- sensors: Sensor data processing (LiDAR, Camera, IMU, Force)
- vision: Vision processing and object detection
- navigation: SLAM and path planning
- audio: Audio processing and voice interaction
- common: Common utilities and helper functions
"""

__version__ = "0.1.0"
__author__ = "Claudia Development Team"
__description__ = "Claudia Intelligent Quadruped Robot System"

# Version information
VERSION = __version__
AUTHOR = __author__
DESCRIPTION = __description__

# Project configuration
PROJECT_NAME = "claudia"
ROBOT_MODEL = "Unitree Go2 R&D Plus"
SUPPORTED_ROS_VERSION = "ROS2 Foxy"
TARGET_PLATFORM = "NVIDIA Jetson Orin NX"
