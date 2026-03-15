# policy_neuro_motion_tracking

StepIt plugin for motion-tracking related modules in the neuro policy pipeline.

### Prerequisites

Install the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library according to the
[official guide](https://stack-of-tasks.github.io/pinocchio/download.html).
If ROS is installed, Pinocchio can be installed via `apt`:

```shell
sudo apt install ros-${ROS_DISTRO}-pinocchio
```

### Provided Factories

`stepit::neuro_policy::Module`:

- `forward_kinematics`: computes whole-body local and global link poses from URDF and joint states.
- `motion_alignment`: aligns stacked target orientations and optional stacked target positions to the robot pose using a configurable reference frame index.
- `motion_player`: loads frame-wise trajectory arrays and publishes configured fields each control step.
- `relative_ori`: computes relative orientation and 6D rotation representation between the current orientation and target orientations.
- `relative_pos`: converts target positions to the current frame.
