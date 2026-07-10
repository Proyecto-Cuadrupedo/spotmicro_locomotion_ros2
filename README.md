# SpotMicro Locomotion ROS 2

This repository adds a ROS 2 Jazzy locomotion layer for the simulated SpotMicro model in `spotmicro_description` using a vendored ROS 2 port of the CHAMP quadruped controller framework.

The repository is intentionally separate from the robot description. `spotmicro_description` owns the URDF, Gazebo plugins, sensors, worlds, simulation launch, and ROS-Gazebo bridge for the model. `spotmicro_locomotion_ros2` owns the vendored CHAMP packages, SpotMicro CHAMP joint/link/gait configuration, the CHAMP control launch, and the bridge that converts CHAMP joint trajectories into SpotMicro Gazebo force commands.

## Runtime Pipeline

```text
/cmd_vel or /body_pose
  -> champ_base/quadruped_controller_node
  -> /spotmicro/champ/joint_targets
  -> spotmicro_champ_bridge
  -> /front_left_shoulder_cmd, /front_left_leg_cmd, ...
  -> ros_gz_bridge from spotmicro_description
  -> Gazebo /model/spotmicro/joint/<joint_name>/cmd_force
```

## Build

Build the SpotMicro control package with its vendored CHAMP packages:

```bash
colcon build --packages-up-to spotmicro_locomotion_ros2
source install/setup.bash
```

Do not use `--packages-select spotmicro_locomotion_ros2` for the first build unless `champ`, `champ_msgs`, and `champ_base` are already built and sourced in `install/`. `--packages-select` skips dependencies, so colcon will fail while looking for their generated `package.sh` files.

The active ROS 2 CHAMP packages are vendored in this repository as `champ`, `champ_msgs`, and `champ_base`. The external `unitree_go2_ros2` checkout is not required for this package and is ignored in this workspace.

## Launch Order

Build and source the workspace, then start the simulation from the description package:

```bash
ros2 launch spotmicro_description sim.launch.py
```

In a second terminal, source the same workspace and start the CHAMP control logic from this package:

```bash
ros2 launch spotmicro_locomotion_ros2 spotmicro_champ_control.launch.py
```

Send walking velocity commands with any `geometry_msgs/msg/Twist` publisher on `/cmd_vel`, for example:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Send body pose commands as `geometry_msgs/msg/Pose` on `/body_pose` to use CHAMP's pose control path.

## Important Files

- `champ`, `champ_msgs`, `champ_base`: vendored ROS 2 CHAMP controller packages.
- `spotmicro_control/launch/spotmicro_champ_control.launch.py`: starts CHAMP control and the force bridge. It expects the `spotmicro_description` simulation to already be running.
- `spotmicro_control/config/joints/joints.yaml`: maps CHAMP leg joints to the SpotMicro URDF actuated joints.
- `spotmicro_control/config/links/links.yaml`: maps CHAMP leg links to the SpotMicro URDF kinematic chain.
- `spotmicro_control/config/gait/gait.yaml`: conservative initial gait parameters for the smaller SpotMicro model.
- `spotmicro_control/src/spotmicro_locomotion_ros2/spotmicro_champ_bridge.py`: PD force bridge from CHAMP joint targets to Gazebo force command topics.

## Tuning

The first tuning point is the bridge gains in the launch file:

```yaml
initial_joint_positions: [0.0, 0.75, -1.45] * 4
startup_stand_duration: 2.0
kp: [4.0, 8.0, 7.0, 4.0, 8.0, 7.0, 4.5, 9.0, 8.0, 4.5, 9.0, 8.0]
kd: [0.12, 0.18, 0.16, 0.12, 0.18, 0.16, 0.14, 0.22, 0.20, 0.14, 0.22, 0.20]
max_force: [2.2, 4.0, 4.0, 2.2, 4.0, 4.0, 2.5, 4.8, 4.8, 2.5, 4.8, 4.8]
```

The bridge holds `initial_joint_positions` briefly before accepting CHAMP trajectories, which gives the simulated robot a standard standing pose instead of immediately chasing walking targets. Each gain or effort value can be a single scalar applied to all 12 joints or a 12-value list in the same order as `spotmicro_control/config/joints/joints.yaml`. Start with small velocities on `/cmd_vel` and increase `kp` or `max_force` gradually only after the simulated joints track smoothly.

For the first walking tune, `spotmicro_control/config/gait/gait.yaml` uses a small negative `com_x_translation`, slower velocities, a longer stance duration, and a taller swing. This shifts some load away from the rear pair and gives the rear legs more time and clearance to enter swing.

If a joint moves in the opposite direction from CHAMP's target, set the corresponding value in the bridge `joint_signs` parameter to `-1.0`.
