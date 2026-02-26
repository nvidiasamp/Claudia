"""Minimal setup.py shim for pip < 21.3 editable install compatibility.

pip 20.0.2 (Ubuntu 20.04 default) does not support PEP 660 editable installs
from pyproject.toml alone. This shim delegates all configuration to pyproject.toml
via setuptools, which reads the [project] table automatically (setuptools >= 61.0).

Usage: pip install -e .
"""
from setuptools import setup

setup()
