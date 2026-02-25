#!/usr/bin/env python3
"""
LED Control System Test Framework
Task 6.5: Comprehensive Testing, Validation, and Performance Optimization

This module provides complete LED control system testing capabilities, including:
- Functional validation tests
- Performance benchmark tests
- Stress and stability tests
- Visual verification tools
- Automated test reports
"""

__version__ = "1.0.0"

# Export main test classes and utilities
from .led_test_base import LEDTestBase
from .test_config import LEDTestConfig
from .data_collector import LEDTestDataCollector

__all__ = [
    'LEDTestBase',
    'LEDTestConfig',
    'LEDTestDataCollector',
]