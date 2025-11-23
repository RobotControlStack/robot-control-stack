# Changelog

## v0.5.2 (2025-10-09)

### Features
- Added OMPL example and cleaned up OMPL code.

## v0.5.1 (2025-09-29)

### Fixes
- Fixed RCS versioning in extensions.

## v0.5.0 (2025-09-26)

### Features
- **Extensions**: Refactored Robotics Library IK into its own extension.
- **Simulation**: Added support for async and realtime mode in SimConfig.
- **Environment**: Added new environment creators for diffpol and agent evaluation.
- **Hardware**: Added digital twin support for xArm and initial support for SO101.
- **Docker**: Added full Docker support with GPU acceleration.
- **Kinematics**: Added Pinocchio IK support with MJCF.
- **Calibration**: Added full calibration support and cache for RealSense.

### Fixes
- Fixed async joint control mode.
- Fixed random object orientation setting.
- Resolved various simulation issues (joint/actuator confusion, robot IDs).

## v0.4.0 (2025-05-12)

### Features
- **Recording**: Added HDF5 recorder wrapper with gzip compression.
- **Teleoperation**: Added async support for teleoperation and webcam live viewer.
- **Environment**: Added collision guard and random cube placement wrapper.
- **Camera**: Added video recording support and rate limiter.

### Fixes
- Fixed FR3 desk errors.
- Improved environment type assertions and tests.
- Fixed simulation GUI rendering limits.

## v0.3.1 (2024-10-02)

### Fixes
- Fixed optional IK bug and FR3 example.

## v0.3.0 (2024-10-02)

### Features
- **Simulation**: Added interactive sim viewer in a separate process.
- **GUI**: Refactored GUI with base class and added MuJoCo UI library.

### Fixes
- Fixed missing depth data in camera environment.

## v0.2.2 (2024-10-01)

### Features
- Added depth data to `CameraSetWrapper`.

### Fixes
- Fixed RGB+Depth mode in camera environment.

## v0.2.1 (2024-09-13)

### Fixes
- Fixed imports and max movement in examples.

## v0.2.0 (2024-09-13)

### Features
- **Teleoperation**: Added keyboard-based robot teleoperation.
- **Camera**: Added ring buffer for hardware cameras.
- **Environment**: Added collision guard environment and parameterizable max movement.
- **Tools**: Added live plotter for robot poses.

### Fixes
- Fixed desk path issues.
- Fixed camera thread checks.
- Resolved various linting and type hinting issues.

## v0.1.0 (2024-06-28)

### Features
- Initial release of RCS.
- **Environment**: Added CameraSet Gym Env.
- **Hardware**: Added support for RealSense cameras and basic robot/gripper interfaces.
- **CI**: Added CI pipeline with linting and testing.
