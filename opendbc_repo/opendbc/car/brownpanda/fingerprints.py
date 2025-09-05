# =============================================================================
# Module      : BrownPanda CAN Fingerprints
# File        : fingerprints.py
# Brief       : Define CAN fingerprints for model identification
# Author      : sunnypilot
# Version     : 1.0
# Last Update : 2025-08-27
# =============================================================================

from opendbc.car.brownpanda.values import CAR

# CAN fingerprint definitions for vehicle model identification
# -----------------------------------------------------------------------------
# [Section] Model Fingerprints
# [Brief ] Message ID to length mappings for identification
# -----------------------------------------------------------------------------
FINGERPRINTS = {
  # BYD ATTO3 - Mid-size SUV EV (identifier: message 112)
  CAR.BYD_ATTO3: [
    {
      # Core vehicle state messages
      1760: 6,   # pandaMessage: Vehicle and device identification
      1761: 8,   # userCommand: Driver inputs and button presses
      1762: 8,   # driverState: Driver monitoring and attention tracking
      1763: 8,   # carState: Primary vehicle dynamics (speed, gear, cruise)
      1764: 8,  # carState2: Extended state (doors, charging, belts)
      1765: 8,  # wheelSensor: Individual wheel speed sensors
      1766: 6,  # adasState: Camera-based lead vehicle detection
      1767: 8,  # chassisState: Stability control and chassis systems
      1768: 8,  # powertrainState: Motor/engine and energy management
      1769: 8,  # imuSensor: Inertial measurement unit data

      # Control command messages
      1770: 8,  # longCommand: Primary longitudinal control (50Hz)
      1771: 8,  # longCommand2: Advanced longitudinal features (25Hz)
      1772: 8,  # latCommand: Steering angle/torque control (100Hz)
      1777: 8,  # dispCommand: Display and UI control (10Hz)
      1775: 8,  # visionInfo: Road analysis and environment data (50Hz)

      # Model identification
      1773: 6,  # modelIdAtto3: BYD ATTO3 specific identifier
    },
  ],

  # BYD DOLPHIN - Compact city EV (identifier: message 113)
  CAR.BYD_DOLPHIN: [
    {
      # Core vehicle state messages
      1760: 6,   # pandaMessage: Vehicle and device identification
      1761: 8,   # userCommand: Driver inputs and button presses
      1762: 8,   # driverState: Driver monitoring and attention tracking
      1763: 8,   # carState: Primary vehicle dynamics (speed, gear, cruise)
      1764: 8,  # carState2: Extended state (doors, charging, belts)
      1765: 8,  # wheelSensor: Individual wheel speed sensors
      1766: 6,  # adasState: Camera-based lead vehicle detection
      1767: 8,  # chassisState: Stability control and chassis systems
      1768: 8,  # powertrainState: Motor/engine and energy management
      1769: 8,  # imuSensor: Inertial measurement unit data

      # Control command messages
      1770: 8,  # longCommand: Primary longitudinal control (50Hz)
      1771: 8,  # longCommand2: Advanced longitudinal features (25Hz)
      1772: 8,  # latCommand: Steering angle/torque control (100Hz)
      1777: 8,  # dispCommand: Display and UI control (10Hz)
      1775: 8,  # visionInfo: Road analysis and environment data (50Hz)

      # Model identification
      1774: 4,  # modelIdDolphin: BYD DOLPHIN specific identifier
    },
  ],

  # DEEPAL S05 - Performance sedan EV (identifier: message 114)
  CAR.DEEPAL_S05: [
    {
      # Core vehicle state messages
      1760: 6,   # pandaMessage: Vehicle and device identification
      1761: 8,   # userCommand: Driver inputs and button presses
      1762: 8,   # driverState: Driver monitoring and attention tracking
      1763: 8,   # carState: Primary vehicle dynamics (speed, gear, cruise)
      1764: 8,  # carState2: Extended state (doors, charging, belts)
      1765: 8,  # wheelSensor: Individual wheel speed sensors
      1766: 6,  # adasState: Camera-based lead vehicle detection
      1767: 8,  # chassisState: Stability control and chassis systems
      1768: 8,  # powertrainState: Motor/engine and energy management
      1769: 8,  # imuSensor: Inertial measurement unit data

      # Control command messages
      1770: 8,  # longCommand: Primary longitudinal control (50Hz)
      1771: 8,  # longCommand2: Advanced longitudinal features (25Hz)
      1772: 8,  # latCommand: Steering angle/torque control (100Hz)
      1777: 8,  # dispCommand: Display and UI control (10Hz)
      1775: 8,  # visionInfo: Road analysis and environment data (50Hz)

      # Model identification
      1776: 8,  # modelIdDeepal: DEEPAL S05 specific identifier
    },
  ],
}

# Compatibility helper removed; use standard car interface introspection if needed.
