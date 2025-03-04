Delto Gripper-3F ROS2 Driver
====================

This package provides a ROS2 driver for the Delto Gripper-3F, allowing for integration with robotic systems using ROS2.

Key Features
------------
- Supports Delto Gripper-3F control via ROS2 topics, services, and actions.

- Includes ROS2 control integration with hardware interface and controllers.

- Provides RViz visualization support for real-time monitoring.

- Offers fake mode for simulation and debugging.

Prerequisites
-------------
- ROS2 (humble)

- Delto Gripper-3F hardware

- Python dependencies:
  
  - `rclpy`
  
  - `trajectory_msgs`
  
  - `control_msgs`
  
  - `std_msgs`
  
  - `sensor_msgs`

Usage
-----

Launching the Driver
~~~~~~~~~~~~~~~~~~~~
To launch the driver and connect to the gripper, use the following commands:

**Visualize using RViz2:**

.. code-block:: bash

   ros2 launch delto_3f_driver delto_3f_bringup.launch.py delto_ip:=192.168.0.112 delto_port:=502 delto_id:=255 launch_rviz:=true

**Optional Parameters:**

- `delto_ip`: IP address of the gripper (default: `169.254.186.72`)

- `delto_port`: Port for the gripper communication (default: `502`)

- `delto_id`: Modbus slave ID for the gripper (default: `1`)

- `launch_rviz`: Launch RViz visualization (default: `true`)

- `fake_mode`: Enable simulation mode without hardware (default: `false`)

ROS2 Control Integration
~~~~~~~~~~~~~~~~~~~~~~~~~
To launch the Delto Gripper-3F driver with ROS2 Control integration:

.. code-block:: bash

   ros2 launch delto_3f_driver delto_3f_ros2_control.launch.py delto_ip:=192.168.0.112 delto_port:=502 

**Optional Parameters:**

- `delto_ip`: IP address of the Gripper (default: `169.254.186.72`)

- `delto_port`: Port for Gripper communication (default: `502`)


Topics
------
The driver communicates via the following topics:

**Published Topics:**

- `/gripper/joint_states` (`sensor_msgs/JointState`): Publishes the current joint states.

**Subscribed Topics:**

- `/gripper/target_joint` (`std_msgs/Float32MultiArray`): Target joint positions (in radians).

- `/gripper/grasp_mode` (`std_msgs/Int32`): Grasp mode commands:
  
  - `0`: Release.
  
  - `1`: 3F Grasp.
  
  - `2~4`: 2F Grasp.
  
  - `5`: Parallel Grasp.
 
  - `6`: Envelope Grasp.

more topic details can be found in the `delto_3f_driver` package.

Controllers
-----------
The Delto Gripper-3F ROS2 driver uses the following controllers:

- `joint_state_broadcaster`: Broadcasts joint states for visualization and monitoring.

- `delto_3f_controller`: Manages joint trajectory commands for the gripper.

Example Usage in Python
-----------------------

Set target joint positions:

.. code-block:: python

   from std_msgs.msg import Float32MultiArray

   target_joint = [0.0] * 12  # Target joint positions (12 joints)
   send_msg = Float32MultiArray()
   send_msg.data = target_joint

   # Publish the message to 'gripper/target_joint'

Set grasp mode:

.. code-block:: python

   from std_msgs.msg import Int32

   grasp_cmd = Int32()
   grasp_cmd.data = 1  # 1: 3F Grasp.

   # Publish the message to 'gripper/grasp_mode'

Customization
-------------
You can customize the behavior of the Delto Gripper-3F driver using the launch file parameters:

**Bringup Parameters (`delto_3f_bringup.launch.py`):**

- `ip`: IP address of the gripper.

- `port`: Port for communication.

- `id`: Modbus slave ID.

- `launch_rviz`: Enable or disable RViz visualization.

- `fake_mode`: Enable fake mode for testing.


Contributing
------------
For bug reports or feature requests, please visit the `GitHub Repository <https://github.com/Tesollo-Delto/DELTO_ROS2>`_.

delto\_3f\_driver Package
-------------------------

.. automodule:: delto_3f_driver.delto_3f_driver
   :members:
   :show-inheritance:
