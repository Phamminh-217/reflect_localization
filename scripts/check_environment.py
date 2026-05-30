#!/usr/bin/env python3
"""Check the basic Python and ROS-related environment for the project."""

import sys


def check_import(module_name: str) -> None:
    """Check whether a Python module can be imported.

    Args:
        module_name: Name of the module to import.

    Raises:
        ImportError: If the module cannot be imported.
    """
    __import__(module_name)
    print(f"[OK] {module_name}")


def main() -> None:
    """Run environment checks."""
    print("Python executable:", sys.executable)
    print("Python version:", sys.version)

    required_modules = [
        "numpy",
        "scipy",
        "sklearn",
        "matplotlib",
        "yaml",
        "pandas",
        "pytest",
    ]

    for module_name in required_modules:
        check_import(module_name)

    try:
        check_import("rosbag")
    except ImportError:
        print("[WARN] rosbag not found. Did you source ROS Noetic?")
        print("       Run: source /opt/ros/noetic/setup.bash")


if __name__ == "__main__":
    main()
