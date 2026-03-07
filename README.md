# cartesian_trajectory_planning_private

Private repo for the Cartesian trajectory planning fro the advanced robotics course

A full tutorial for a 6 DOF robot for intermediate ROS 2 users.

It consists of the following:

* bringup: launch files and ros2_controller configuration
* controller: a controller for the 6-DOF robot
* description: the 6-DOF robot description
* hardware: ros2_control hardware interface
* reference_generator: A KDL-based reference generator for a fixed trajectory

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html).

## Luanch trajectory generation demo

In one terminal, run:

```bash
ros2 launch cartesian_trajectory_planning r6bot_controller.launch.py
```

In another terminal, run:

```bash
ros2 launch cartesian_trajectory_planning send_trajectory.launch.py
```

## show EE trail in Rviz

* Go to RobotModel>Links>tool0 (or the link that refers to the EE).
* Habilitate Show Trail.
