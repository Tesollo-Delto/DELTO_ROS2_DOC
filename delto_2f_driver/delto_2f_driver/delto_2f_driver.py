#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
delto_2f_driver
===============

A ROS2 node to control and read data from a Delto 2-finger gripper
via Modbus TCP communication. This module defines the `DeltoROSDriver` class.

Example:
    To run this node, execute:

    .. code-block:: bash

        ros2 run delto_2f_driver delto_2f_driver
"""

import math
import time
import threading
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Int32
from ros_gz_interfaces.msg import Float32Array
from sensor_msgs.msg import JointState

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from delto_utility import delto_2f_modbus_TCP as delto_TCP


class DeltoROSDriver(Node):
    """
    A ROS2 Node to interface with the Delto 2-finger gripper via Modbus TCP.

    This class handles:
    - Establishing a connection to the gripper
    - Subscribing to topics for force settings, open/close positions, and grasp commands
    - Publishing current gripper states (position, force levels) at a defined rate

    :param name: The name of the node (defaults to 'delto_2f_driver').
    :type name: str, optional
    """

    def __init__(self):
        """
        Initialize the DeltoROSDriver node.

        The constructor sets up:
        - ROS2 parameters (e.g., IP, port, slaveID)
        - Publishers and subscribers for gripper commands and states
        - Timers for data retrieval and publishing joint states
        """
        super().__init__('delto_2f_driver')

        qos_profile = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.RELIABLE)

        self.declare_parameter('ip', "192.254.186.62")
        self.declare_parameter('port', 10000)
        self.declare_parameter('slaveID', 0)
        self.declare_parameter('dummy', False)

        self.current_position = 0
        self.electric_current = 0
        self.low_force = 0
        self.high_force = 0

        self.delto_client = delto_TCP.Communication()
        self.stop_thread = False
        self.lock = threading.Lock()

        # Publish rate in Hz
        self.publish_rate = 20

        # Publishers
        self.jointstate_pub = self.create_publisher(JointState, 'gripper/joint_states', qos_profile)
        self.low_force_pub = self.create_publisher(Int32, 'gripper/status/low_force', qos_profile)
        self.high_force_pub = self.create_publisher(Int32, 'gripper/status/high_force', qos_profile)
        self.open_position_pub = self.create_publisher(Int32, 'gripper/status/open_position', qos_profile)
        self.close_position_pub = self.create_publisher(Int32, 'gripper/status/close_position', qos_profile)

        # Subscribers
        self.low_force_sub = self.create_subscription(Int32, 'gripper/low_force', self.low_force_callback, qos_profile)
        self.high_force_sub = self.create_subscription(Int32, 'gripper/high_force', self.high_force_callback, qos_profile)
        self.open_position_sub = self.create_subscription(Int32, 'gripper/open_position', self.open_position_callback, qos_profile)
        self.close_position_sub = self.create_subscription(Int32, 'gripper/close_position', self.close_position_callback, qos_profile)
        self.grasp_sub = self.create_subscription(Int32, 'gripper/grasp', self.grasp_callback, qos_profile)

        # Timer for retrieving gripper data
        self.get_data_timer = self.create_timer(1.0 / self.publish_rate, self.get_data_timer_callback)

        self.joint_state_msg = JointState()

    def connect(self) -> bool:
        """
        Connect to the Delto gripper with provided IP, port, and slave ID.

        :return: True if the connection is successful, False otherwise.
        :rtype: bool
        """
        self.get_logger().info("Connecting to the Delto gripper...")
        return self.delto_client.connect(
            self.get_parameter('ip').value,
            self.get_parameter('port').value,
            self.get_parameter('slaveID').value
        )

    def joint_state_publisher(self) -> None:
        """
        Publish the current joint state (gripper opening distance).
        The position is published in meters (current_position * 0.001).
        """
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.name = ['Distance']
        self.joint_state_msg.position = [self.current_position * 0.001]
        self.jointstate_pub.publish(self.joint_state_msg)

    def get_open_position(self) -> int:
        """
        Get the current open position setting of the gripper.

        :return: The open position in some integer unit (depends on device).
        :rtype: int
        """
        return self.open_position

    def get_close_position(self) -> int:
        """
        Get the current close position setting of the gripper.

        :return: The close position in some integer unit (depends on device).
        :rtype: int
        """
        return self.close_position

    def get_current_position(self) -> float:
        """
        Get the real-time position of the gripper (read from the device).

        :return: The current gripper position.
        :rtype: float
        """
        return self.current_position

    def get_electric_current(self) -> float:
        """
        Get the electric current value read from the gripper.

        :return: The gripper’s electric current.
        :rtype: float
        """
        return self.electric_current

    def set_ip(self, ip: str) -> None:
        """
        Set a new IP address for the Modbus TCP client.

        :param ip: The IP address to set.
        :type ip: str
        """
        self.delto_client.set_ip(ip)

    def grasp(self, is_grasp: int) -> None:
        """
        Send a grasp or release command to the gripper.

        :param is_grasp: 1 to grasp, 0 to release.
        :type is_grasp: int
        """
        self.delto_client.grasp(bool(is_grasp))

    def set_open_position(self, position: int) -> None:
        """
        Set the open position of the gripper.

        :param position: The open position in integer units.
        :type position: int
        """
        self.open_position = position
        self.delto_client.set_open_position(self.open_position)

    def set_close_position(self, position: int) -> None:
        """
        Set the close position of the gripper.

        :param position: The close position in integer units.
        :type position: int
        """
        self.close_position = position
        self.delto_client.set_close_position(self.close_position)

    def set_high_force(self, force: int) -> None:
        """
        Set the high force threshold of the gripper.

        :param force: The force value for high threshold.
        :type force: int
        """
        self.high_force = force
        self.delto_client.set_high_force(self.high_force)

    def set_low_force(self, force: int) -> None:
        """
        Set the low force threshold of the gripper.

        :param force: The force value for low threshold.
        :type force: int
        """
        self.low_force = force
        self.delto_client.set_low_force(self.low_force)

    def high_force_callback(self, msg: Int32) -> None:
        """
        Callback function for the 'gripper/high_force' topic.

        :param msg: ROS2 message containing the high force value to set.
        :type msg: Int32
        """
        self.set_high_force(msg.data)

    def low_force_callback(self, msg: Int32) -> None:
        """
        Callback function for the 'gripper/low_force' topic.

        :param msg: ROS2 message containing the low force value to set.
        :type msg: Int32
        """
        self.set_low_force(msg.data)

    def open_position_callback(self, msg: Int32) -> None:
        """
        Callback function for the 'gripper/open_position' topic.

        :param msg: ROS2 message containing the desired open position.
        :type msg: Int32
        """
        self.set_open_position(msg.data)

    def close_position_callback(self, msg: Int32) -> None:
        """
        Callback function for the 'gripper/close_position' topic.

        :param msg: ROS2 message containing the desired close position.
        :type msg: Int32
        """
        self.set_close_position(msg.data)

    def grasp_callback(self, msg: Int32) -> None:
        """
        Callback function for the 'gripper/grasp' topic.

        :param msg: ROS2 message indicating grasp (1) or release (0).
        :type msg: Int32
        """
        self.grasp(msg.data)

    def get_data_timer_callback(self) -> None:
        """
        Timer callback to periodically retrieve data from the gripper and
        publish relevant ROS messages (joint state, force thresholds, etc.).
        """
        self.lock.acquire()
        data = self.delto_client.get_data()
        self.lock.release()

        # Example data layout check (if you know data structure)
        # if len(data) != 5:
        #     return

        self.current_position = float(data[0])
        self.electric_current = float(data[1])

        self.high_force = self.delto_client.get_high_force()
        self.low_force = self.delto_client.get_low_force()

        force_msg = Int32()
        force_msg.data = self.low_force
        self.low_force_pub.publish(force_msg)

        force_msg.data = self.high_force
        self.high_force_pub.publish(force_msg)

        self.joint_state_publisher()


def main(args=None) -> None:
    """
    Main entry point for the Delto 2F ROS driver node.

    Initializes the node, attempts to connect to the gripper,
    and starts the multi-threaded executor to handle callbacks.

    :param args: Command-line arguments (default: None).
    :type args: list or None
    """
    rclpy.init(args=args)

    delto_driver = DeltoROSDriver()
    connect = delto_driver.connect()

    if not connect:
        delto_driver.get_logger().error("Network connection failed.")
        return

    time.sleep(0.1)

    delto_driver.get_logger().info("delto_2f_driver initialized")

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(delto_driver)
    executor.spin()
    executor.shutdown()


if __name__ == '__main__':
    main()
