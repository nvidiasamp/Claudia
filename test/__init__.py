"""
Claudia Robot Project Test Suite

This package contains all test code for the Claudia intelligent quadruped robot system:
- unit/: Unit tests
- integration/: Integration tests
- hardware/: Hardware-related tests
- utils/: Test utility functions
"""

__version__ = "1.0.0"
__author__ = "Claudia Project Team"

# Import commonly used test utilities
import sys
import os
from pathlib import Path

# Add project root directory to Python path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))