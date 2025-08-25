# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Common Development Commands

**Build System:**
- `scons` - Build the entire project using the SCons build system
- `scons -j8` - Build with 8 parallel jobs (adjust based on CPU cores)
- `scons --compile_db` - Generate compilation database (compile_commands.json)

**Testing:**
- `pytest` - Run the test suite using pytest
- `pytest -xvs` - Run tests with verbose output and stop on first failure
- `pytest selfdrive/car/tests/` - Run tests for specific modules
- `ruff check .` - Run linting checks using ruff
- `ruff format .` - Format code using ruff

**Python Environment:**
- Uses Python 3.11+ as specified in pyproject.toml
- Uses Poetry for dependency management
- Major dependencies: numpy, cython, capnp, onnx, zmq, requests

## Repository Architecture

**sunnypilot** is a fork of comma.ai's openpilot - an open-source Advanced Driver Assistance System (ADAS) that runs on comma devices (comma two/three) to provide lane keeping, adaptive cruise control, and other driving assistance features.

### Core System Components

**System Manager (`system/manager/`):**
- `manager.py` - Main process manager that orchestrates all other processes
- `process_config.py` - Defines which processes run and their configurations
- Manages process lifecycle, restarts, and health monitoring

**Vehicle Interface (`selfdrive/car/`):**
- Brand-specific implementations for 250+ supported cars (Honda, Toyota, Hyundai, etc.)
- Each brand has: `interface.py`, `carstate.py`, `carcontroller.py`, `values.py`, `fingerprints.py`
- `car_helpers.py` - Common functions across all vehicle interfaces
- `fingerprints.py` - Vehicle identification and CAN message signatures

**Controls (`selfdrive/controls/`):**
- `controlsd.py` - Main control loop that coordinates lateral and longitudinal control
- `lib/latcontrol*.py` - Lateral (steering) control implementations (PID, torque, angle)
- `lib/longcontrol.py` - Longitudinal (speed/braking) control
- `lib/alertmanager.py` - Safety alerts and warnings system

**Vision and ML (`selfdrive/modeld/`):**
- Neural network model execution for vision-based driving
- `modeld.py` - Main model daemon for path planning
- `models/` - Contains trained model files and metadata
- Supports ONNX runtime on x86_64 and specialized hardware on devices

**Communication (`cereal/`, `msgq/`):**
- Inter-process communication using Cap'n Proto serialization
- `cereal/` - Message schemas and definitions
- `msgq/` - Message queue implementation with ZeroMQ backend

### sunnypilot-Specific Features

**Modified Assistive Driving Safety (MADS):**
- Allows independent operation of Lane Centering (ALC) and Adaptive Cruise Control (ACC)
- Enhanced safety model that maintains comma.ai's safety standards
- Custom button mappings for different vehicle brands

**Enhanced Speed Control:**
- Vision-based Turn Speed Control (V-TSC) using AI model predictions
- Map-based Turn Speed Control (M-TSC) using OpenStreetMap data
- Speed Limit Control (SLC) from map data and vehicle interface
- Custom longitudinal control for supported vehicles

**Dynamic Lane Profile (DLP):**
- Dynamically switches between "Laneful" and "Laneless" driving modes
- Based on lane recognition confidence and road conditions

## Key File Structure

- `SConstruct` - Main SCons build configuration
- `pyproject.toml` - Python project configuration, dependencies, and tool settings
- `selfdrive/` - Core driving assistance logic
- `system/` - System-level services (camera, logging, hardware interfaces)
- `common/` - Shared utilities and helper functions
- `opendbc/` - CAN bus message definitions and DBC files
- `body/` - Firmware for comma body (robotics platform)
- `panda/` - Firmware and interface for comma panda (CAN interface device)
- `tools/` - Development and debugging utilities

## Development Notes

**Code Style:**
- Python code follows PEP 8 with ruff formatting (160 char line limit)
- Uses type hints extensively with mypy static type checking
- Cython extensions for performance-critical code

**Testing Structure:**
- Tests located alongside source code in `tests/` subdirectories
- Uses pytest framework with custom fixtures for hardware simulation
- Separate test categories: unit tests, integration tests, and hardware-specific tests

**Safety and Compliance:**
- All modifications must maintain comma.ai's safety standards
- Panda safety code enforces hardware-level safety constraints
- No driver monitoring reductions or safety bypasses permitted

**Hardware Support:**
- Primarily targets comma three (newer) and comma two (legacy) devices
- Cross-platform support for development on x86_64, aarch64, and Darwin
- Uses specialized hardware accelerators on device for ML inference