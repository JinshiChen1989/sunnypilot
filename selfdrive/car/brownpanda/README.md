# BrownPanda Vehicle Interface

OpenPilot integration for BrownPanda platforms (e.g., BYD ATTO3, BYD DOLPHIN, DEEPAL S05). This module is radarless and uses a single-bus CAN mapping, with clear separation between parsing, configuration, and control.

## Architecture Flow
```
CAN (single bus) → CarState (parse → cereal.CarState) → CarInterface
                                                   │               │
                                                   ▼               │
                                       openpilot Controls          ▼
CarControl (actuators) ←────────────── planner/controllers     CarController
                                                                │
                                                                ▼
                           CAN out (latCommand | longCommand | longCommand2 | dispCommand | visionInfo)
```

## Buses & Timing
- Bus layout: `brownpandacan.CanBus` maps `pt/radar/lkas` to the same physical bus (radarless design).
- Message cadence (`carcontroller.py`): `latCommand` 100 Hz; `longCommand` 50 Hz; `longCommand2` 25 Hz; `visionInfo` 50 Hz; `dispCommand` 10 Hz.

## Key Files
- `interface.py`: Builds `car.CarParams`, selects lateral mode (angle/torque), enables camera-based longitudinal, sets `BrownPandaFlags`, actuator delays, and basic specs (mass, wheelbase, steer ratio).
- `carstate.py`: Parses CAN with `CANParser` into `cereal.car.CarState` (speed, wheel speeds, steering angle/torque, pedals, gears, cruise, BSM, IMU, basic vision-derived lead info). Uses thresholds from `values.py`.
- `carcontroller.py`: Converts `CarControl` to CAN with accel/steer limits and rate limiting; emits the commands above at fixed rates.
- `brownpandacan.py`: Packs CAN payloads for BrownPanda messages and defines the single-bus `CanBus` mapping.
- `values.py`: Defines `CAR` enums, `DBC` map, `CarControllerParams` (limits, rate/angle deltas), and feature flags `BrownPandaFlags`.
- `fingerprints.py`: CAN ID→len fingerprints for platform detection.

## Parameters & Tuning
- Longitudinal: bounds and pedal/torque scaling in `values.py::CarControllerParams`; PID accel limits returned by `interface.get_pid_accel_limits`.
- Lateral: selected in `interface.py` based on flags; tuning set in `ret.lateralTuning` (PID or torque mode).
- Vehicle specs: mass/wheelbase/steer ratio loaded per platform from `values.py`-referenced configs.

## Development & Testing
- Build natives: `scons -j$(nproc)`
- Run tests (module): `pytest selfdrive/car/brownpanda -m "not slow"`
- Lint/format/types: `ruff check . && ruff format . && mypy .`
- Example targeted test: `pytest selfdrive/car/brownpanda/test_interface.py::test_params`

## Contributing
- See `AGENTS.md` for repository-wide guidelines (structure, commands, style, PR expectations).
- For car changes, update DBC/fingerprints as needed and include a short route or log segment for validation.

