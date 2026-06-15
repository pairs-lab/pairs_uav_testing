# pairs_uav_testing

Integration-testing support for the PAIRS multirotor-UAV autonomy stack. It provides a small
C++ test library and reusable launch files that spin up a full UAV system in the simulator,
then drive scripted maneuvers (takeoff, goto, relative goto) to verify the control,
estimation, and management layers end to end. Other PAIRS packages depend on it as their
`test_depend` to build their own rostests on top of the same harness.

## Contents

- `test_generic` library — base `TestGeneric` / `UAVHandler` classes that drive a UAV through a scripted test
- `pairs_uav_system.launch` — brings up the full PAIRS UAV system (core, autostart) for a test
- `pairs_simulator.launch` — launches the PAIRS multirotor simulator
- Example rostests under `test/`: `takeoff`, `goto`, `goto_relative`
- Default custom / world / network / simulator configs under `config/`

## Branches

- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 1 Noetic)

```bash
sudo apt install ros-noetic-pairs-uav-testing
```

## Usage

Bring up the full UAV system for testing:

```bash
roslaunch pairs_uav_testing pairs_uav_system.launch
```

Or start just the simulator:

```bash
roslaunch pairs_uav_testing pairs_simulator.launch
```

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_uav_testing` package; the original
copyright is retained in [LICENSE](LICENSE).
