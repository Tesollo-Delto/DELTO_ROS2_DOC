Delto Gripper-3F ROS2 Control Configuration
===========================================

We are excited to announce that the **Delto Gripper-3F** supports **ros2_control**! Below is an example configuration for integrating the gripper with a ROS 2 system using the `joint_trajectory_controller` and `joint_state_broadcaster`.

Controller and Broadcaster Configuration
-----------------------------------------

Below is the YAML configuration for the **Delto Gripper-3F** in a `ros2_control` framework:

.. code-block:: yaml

   controller_manager:
     ros__parameters:
       update_rate: 500  # Joint state pub rate

       joint_state_broadcaster:
         type: joint_state_broadcaster/JointStateBroadcaster

       delto_3f_controller:
         type: joint_trajectory_controller/JointTrajectoryController

   delto_3f_controller:
     ros__parameters:
       state_publish_rate: 500.0
       action_monitor_rate: 500.0
       gains:
         F1M1: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F1M2: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F1M3: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F1M4: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F2M1: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F2M2: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F2M3: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F2M4: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F3M1: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F3M2: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F3M3: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}
         F3M4: {p: 1.2, i: 0.0, d: 0.1, i_clamp: 0.0}

       allow_partial_joints_goal: True
       joints:
         - F1M1
         - F1M2
         - F1M3
         - F1M4
         - F2M1
         - F2M2
         - F2M3
         - F2M4
         - F3M1
         - F3M2
         - F3M3
         - F3M4
       command_interfaces:
         - effort
       state_interfaces:
         - position
         - velocity

   joint_state_broadcaster:
     ros__parameters:
       update_rate: 500
       joints:
         - F1M1
         - F1M2
         - F1M3
         - F1M4
         - F2M1
         - F2M2
         - F2M3
         - F2M4
         - F3M1
         - F3M2
         - F3M3
         - F3M4

Explanation
------------

- **`update_rate`**: Sets the rate at which joint states are published (500 Hz for high-frequency updates).

- **Gains**: Proportional (P), Integral (I), and Derivative (D) values for precise control of each joint.

- **`allow_partial_joints_goal`**: Allows for setting goals for a subset of joints.

- **Interfaces**:

  - **Command Interfaces**: Effort-based input.

  - **State Interfaces**: Position and velocity.

How to Use
-----------
1. Copy the above YAML configuration into your `ros2_control` configuration file.
2. Launch the `controller_manager` in your ROS2 system.
3. Use effort-based commands to control the gripper.

For any additional help or suggestions, feel free to reach out to the maintainers or post on our community forums.
