# pairs_uav_testing

Integration-testing support for the PAIRS multirotor-UAV autonomy stack. It provides a small
C++ test library and reusable launch files that spin up a full UAV system in the simulator,
then drive scripted maneuvers (takeoff, goto, relative goto) to verify the control,
estimation, and management layers end to end. Other PAIRS packages depend on it as their
`test_depend` to build their own launch tests on top of the same harness.

## Contents

- `pairs_uav_testing` library — base test classes that drive a UAV through a scripted test
- `pairs_uav_system.launch.py` — brings up the full PAIRS UAV system for a test
- `pairs_multirotor_simulator.launch.py` — launches the PAIRS multirotor simulator
- Example tests under `test/`: `takeoff`, `goto_absolute`, `goto_relative`
- Default custom / world / network / simulator configs under `config/`

## Branches

- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-pairs-uav-testing
```

## Usage

Bring up the full UAV system for testing:

```bash
ros2 launch pairs_uav_testing pairs_uav_system.launch.py
```

Or start just the simulator:

```bash
ros2 launch pairs_uav_testing pairs_multirotor_simulator.launch.py
```

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_uav_testing` package; the original
copyright is retained in [LICENSE](LICENSE).
