Delto Gripper-2F ROS2 Driver
====================

This package provides a ROS2 driver for the Delto Gripper-2F, enabling integration with robotic systems using ROS2.

Key Features
------------
- ROS2 topics and services for controlling the Delto Gripper-2F.
- Joint state feedback for real-time monitoring.
- Customizable gripper operation modes.

Prerequisites
-------------
- ROS2 (Humble)
- DELTO 2F Gripper hardware
- Python dependencies:
  
  - `rclpy`
  - `std_msgs`
  - `sensor_msgs`

Usage
-----

Launching the Driver
~~~~~~~~~~~~~~~~~~~~
To launch the Delto Gripper-2F driver and connect to the gripper, use the following command:

.. code-block:: bash

   ros2 run delto_2f_driver delto_2f_driver --ros-args -p ip:=192.168.0.112 -p port:=10000 -p slaveID:=1

**Optional Parameters:**

- `ip`: IP address of the gripper (default: `169.254.186.72`)
- `port`: Communication port for the gripper (default: `502`)
- `slaveID`: Modbus slave ID for the gripper (default: `1`)

Topics
------
The driver communicates via the following topics:

**Published Topics:**

- **`gripper/joint_states`** (`sensor_msgs/JointState`): Publishes the current joint state of the gripper.

**Subscribed Topics:**

- **`gripper/grasp`** (`std_msgs/Int32`): Controls the gripper's grasping operation:
  
  - `0`: Open the gripper.
  - `1`: Close the gripper.

- **`gripper/low_force`** (`std_msgs/Int32`): Sets the gripper's low-force threshold.

- **`gripper/high_force`** (`std_msgs/Int32`): Sets the gripper's high-force threshold.

- **`gripper/open_position`** (`std_msgs/Int32`): Configures the open position of the gripper.

- **`gripper/close_position`** (`std_msgs/Int32`): Configures the close position of the gripper.

Example Usage in Python
-----------------------

Control the gripper using ROS2 topics:

.. code-block:: python

   from std_msgs.msg import Int32
   import rclpy
   from rclpy.node import Node

   class GripperControl(Node):
       def __init__(self):
           super().__init__('gripper_control')
           self.publisher_ = self.create_publisher(Int32, 'gripper/grasp', 10)

       def open_gripper(self):
           msg = Int32()
           msg.data = 0  # Open the gripper
           self.publisher_.publish(msg)

       def close_gripper(self):
           msg = Int32()
           msg.data = 1  # Close the gripper
           self.publisher_.publish(msg)

   def main(args=None):
       rclpy.init(args=args)
       node = GripperControl()
       node.open_gripper()  # Example: Open the gripper
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

Contributing
------------
For bug reports or feature requests, please visit the `GitHub Repository <https://github.com/Tesollo-Delto/DELTO_ROS2>`_.

delto\_2f\_driver Package
-------------------------

.. automodule:: delto_2f_driver.delto_2f_driver
   :members:
   :show-inheritance:
