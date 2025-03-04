#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Delto 3F Gripper ROS2 Driver
============================

This module provides a ROS2 node for controlling the Delto 3F gripper. It communicates
with the actual hardware via Modbus TCP and offers various messages and actions
to control and monitor the gripper.

Key features:

- Read the current joint angles of the gripper

- Set target joint angles

- Save/load gripper poses to/from internal memory

- Execute a grasp command and switch between different grasp modes

- Update and retrieve P/D gains

- Provide a FollowJointTrajectory action server

Example:

    Below is a basic example of how to run this node via the ``main()`` function:

    .. code-block:: console

        $ ros2 run delto_3f_driver delto_3f_driver

Note:
    This node depends on ROS2 parameters (`ip`, `port`, `slaveID`, `dummy`).  
    In a typical setup, these parameters would be injected via a launch file.
"""

import math
import time
import threading
import sys
import os
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Int32, Bool, Float32MultiArray, Int16MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from delto_utility import delto_modbus_TCP as delto_TCP


class DeltoROSDriver(Node):
    """
    A ROS2 node class for controlling the Delto 3F gripper.

    This class inherits from the ROS2 Node and provides the following key attributes:

    :ivar joint_state_list: List of waypoints (desired JointTrajectory points).
    :vartype joint_state_list: list[float]

    :ivar current_joint_state: Current joint angles of the gripper (in radians).
    :vartype current_joint_state: list[float]

    :ivar target_joint_state: Target gripper joint angles (in radians).
    :vartype target_joint_state: list[float]

    :ivar fixed_joint_state: Fixed joint values (e.g. for locking certain joints).
    :vartype fixed_joint_state: list[int]

    :ivar joint_state_feedback: A JointTrajectoryPoint object used for action feedback.
    :vartype joint_state_feedback: JointTrajectoryPoint

    :ivar vel: (Currently unused) A placeholder for velocity or speed-related parameters.
    :vartype vel: list

    :ivar delto_client: A Modbus TCP client for low-level communication with the gripper hardware.
    :vartype delto_client: delto_TCP.Communication

    :ivar stop_thread: A boolean flag to stop the waypoint moving thread.
    :vartype stop_thread: bool

    :ivar lock: A threading lock for resource synchronization in multi-threaded environments.
    :vartype lock: threading.Lock

    :ivar is_connected: A flag indicating whether the gripper is successfully connected.
    :vartype is_connected: bool

    :ivar publish_rate: Frequency (Hz) at which JointState messages are published.
    :vartype publish_rate: int

    :ivar is_dummy: Whether to operate in a dummy (simulation) mode instead of real hardware.
    :vartype is_dummy: bool

    :ivar jcm_action_server: FollowJointTrajectory action server.
    :vartype jcm_action_server: ActionServer

    :ivar joint_state_pub: Publisher for JointState messages.
    :vartype joint_state_pub: Publisher

    :ivar grasp_sub: Subscription to Bool topic (to trigger grasp operation).
    :vartype grasp_sub: Subscription

    :ivar write_register_sub: Subscription to Int16MultiArray topic (to write directly to registers).
    :vartype write_register_sub: Subscription

    :ivar grasp_mode_sub: Subscription to Int32 topic (to set a grasp mode).
    :vartype grasp_mode_sub: Subscription

    :ivar target_joint_sub: Subscription to Float32MultiArray topic (to set target joint angles).
    :vartype target_joint_sub: Subscription

    :ivar joint_state_timer: Timer for JointState publishing.
    :vartype joint_state_timer: Timer

    :ivar read_joint_timer: Timer for reading current joint angles.
    :vartype read_joint_timer: Timer

    :ivar fixed_joint_sub: Subscription to Int16MultiArray topic (to lock particular joints).
    :vartype fixed_joint_sub: Subscription

    :ivar set_gain_sub: Subscription to Int16MultiArray topic (to set P/D gains).
    :vartype set_gain_sub: Subscription

    :ivar gain_pub: Publisher for Int16MultiArray topic (to respond with current P/D gains).
    :vartype gain_pub: Publisher

    :ivar load_pose_sub: Subscription to Int32 topic (to load a pose from internal storage).
    :vartype load_pose_sub: Subscription

    :ivar save_pose_sub: Subscription to Int32 topic (to save a pose to internal storage).
    :vartype save_pose_sub: Subscription

    :ivar reconnect_timer: Timer to periodically attempt reconnection when the connection is lost.
    :vartype reconnect_timer: Timer

    :ivar reconnect_attempts: Count of how many times reconnection has been attempted.
    :vartype reconnect_attempts: int

    :ivar max_reconnect_attempts: Maximum number of reconnection attempts before giving up.
    :vartype max_reconnect_attempts: int
    """

    def __init__(self):
        """
        Initializes the node, sets up ROS2 publishers/subscribers/action servers,
        and prepares the Modbus TCP client to communicate with the gripper hardware.
        """
        super().__init__('delto_3f_driver')
        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        fast_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
            lifespan=Duration(seconds=1),
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.declare_parameter('ip', "169.254.186.62")
        self.declare_parameter('port', 10000)
        self.declare_parameter('slaveID', 1)
        self.declare_parameter('dummy', False)

        self.joint_state_list = [0.0]*12
        self.current_joint_state = [0.0]*12
        self.target_joint_state = [0.0]*12
        self.fixed_joint_state = [0]*12

        self.joint_state_feedback = JointTrajectoryPoint()
        self.vel = []
        self.delto_client = delto_TCP.Communication()
        self.stop_thread = False
        self.lock = threading.Lock()
        self.is_connected = False
        
        # A very high publish rate may cause issues in sub/pub, set to a safe default
        self.publish_rate = 100
        print('publish rate : ' + str(self.publish_rate))
        
        self.is_dummy = False  # bool(self.get_parameter('dummy').value)
        self.is_connected = False

        # Action Server Setup
        self.jcm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'delto_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Publishers & Subscriptions
        self.joint_state_pub = self.create_publisher(
            JointState, 'gripper/joint_states', fast_qos)
        self.grasp_sub = self.create_subscription(
            Bool, 'gripper/grasp', callback=self.grasp_callback, qos_profile=qos_profile)
        self.write_register_sub = self.create_subscription(
            Int16MultiArray, 'gripper/write_register', self.write_register_callback, qos_profile=qos_profile)
        self.grasp_mode_sub = self.create_subscription(
            Int32, 'gripper/grasp_mode', callback=self.grasp_mode_callback, qos_profile=qos_profile)
        self.target_joint_sub = self.create_subscription(
            Float32MultiArray, 'gripper/target_joint', callback=self.target_joint_callback, qos_profile=qos_profile)

        self.joint_state_timer = self.create_timer(
            1/self.publish_rate, self.timer_callback)
        self.read_joint_timer = self.create_timer(
            1/self.publish_rate, self.read_joint_callback)
        self.fixed_joint_sub = self.create_subscription(
            Int16MultiArray, 'gripper/fixed_joint', self.fixed_joint_callback, qos_profile=qos_profile)
        self.set_gain_sub = self.create_subscription(
            Int16MultiArray, 'gripper/request/gain', self.set_gain_callback, qos_profile=qos_profile)
        self.gain_pub = self.create_publisher(
            Int16MultiArray, 'gripper/response/gain', qos_profile=qos_profile)
        
        self.load_pose_sub = self.create_subscription(
            Int32, 'gripper/load_pose', self.load_pose_callback, qos_profile=qos_profile)
        self.save_pose_sub = self.create_subscription(
            Int32, 'gripper/save_pose', self.save_pose_callback, qos_profile=qos_profile)

        self.reconnect_timer = self.create_timer(1.0, self.reconnect_callback)
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 100

    def load_pose_callback(self, msg):
        """
        Loads a stored gripper pose (index range: 1~30) from internal EEPROM.

        :param msg: The pose index to be loaded
        :type msg: Int32
        :raises: Logs an error if the index is out of valid range (1~30).
        """
        if msg.data < 1 or msg.data > 30:
            self.get_logger().error("Pose index out of range")
            return
        self.delto_client.load_pose(msg.data)
        
    def save_pose_callback(self, msg):
        """
        Saves the current gripper pose to a specified index (range: 1~30) in the internal EEPROM.

        :param msg: The pose index to save to
        :type msg: Int32
        :raises: Logs an error if the index is out of valid range (1~30).
        """
        if msg.data < 1 or msg.data > 30:
            self.get_logger().error("Pose index out of range")
            return
        
        self.delto_client.save_pose(msg.data)
        
    def reconnect_callback(self):
        """
        A periodic callback that attempts to reconnect to the gripper if the connection is lost.
        If :attr:`reconnect_attempts` exceeds :attr:`max_reconnect_attempts`, the node is shut down.
        """
        if not self.is_connected:
            if self.reconnect_attempts < self.max_reconnect_attempts:
                self.get_logger().info(
                    f"Attempting to reconnect (attempt {self.reconnect_attempts + 1}/{self.max_reconnect_attempts})")
                
                try: 
                    self.connect()
                    self.reconnect_attempts += 1
                    
                    if self.is_connected:
                        self.get_logger().info("Reconnected successfully")
                        self.reconnect_attempts = 0
                        
                except Exception as e:
                    self.get_logger().error(f"Failed to reconnect: {e}")
                    self.reconnect_attempts += 1
            else:
                self.get_logger().error("Maximum reconnect attempts reached. Shutting down.")
                self.destroy_node()
        else:
            self.reconnect_attempts = 0

    def set_gain_callback(self, msg):
        """
        Callback function to set P/D gains.  
        The ``msg.data`` is expected to have 24 elements, the first 12 for P gains, and the next 12 for D gains.

        :param msg: Int16MultiArray with 24 elements (12 for P, 12 for D)
        :type msg: Int16MultiArray
        :raises: Logs an error if the array size is not 24.
        """
        if len(msg.data) != 24:
            self.get_logger().error(f"Invalid gain array size: {len(msg.data)}")
            return
        
        self.delto_client.set_pgain(msg.data[0:12])
        self.delto_client.set_dgain(msg.data[12:24])
        pgain = self.delto_client.get_pgain()
        dgain = self.delto_client.get_dgain()

        data = []
        data.extend(pgain)
        data.extend(dgain)
        
        gain_msg = Int16MultiArray()
        gain_msg.data = data
        self.gain_pub.publish(gain_msg)

    def connect(self) -> bool:
        """
        Attempts to connect to the Delto gripper.  
        Internally calls ``self.delto_client.connect()``. If successful, 
        :attr:`is_connected` is set to True.

        :return: True if the connection succeeds, False otherwise.
        :rtype: bool
        """
        if self.is_dummy:
            self.get_logger().info("Dummy mode")
            return True

        self.get_logger().info("Connecting to the delto gripper...")
        is_connected = self.delto_client.connect(
            self.get_parameter('ip').value,
            self.get_parameter('port').value,
            self.get_parameter('slaveID').value
        )
        self.is_connected = is_connected
        return is_connected

    def read_joint_callback(self):
        """
        Reads the current joint angles of the gripper (in degrees) and updates
        :attr:`current_joint_state` (in radians).

        If the connection is lost or an exception occurs, logs an error and sets
        :attr:`is_connected` to False.
        """
        if self.reconnect_attempts > 1:
            return
        
        if not self.is_connected:
            return

        try:
            position_tmp = self.get_position()
            self.current_joint_state = [float(self._deg2rad(x)) for x in position_tmp]
        except Exception as e:
            self.get_logger().error(f"Failed to read joint state: {e}")
            self.is_connected = False

    def write_register_callback(self, msg):
        """
        A callback function to write data directly to specific registers of the module.
        For instance, the Int16MultiArray might be structured as 
        (start_register_address, data1, data2, ...).

        :param msg: The first element is the register address, the subsequent elements are the actual data.
        :type msg: Int16MultiArray
        """
        if not self.is_connected:
            self.get_logger().error("Connection lost")
            return
        
        try:
            self.delto_client.write_registers(msg.data[0], msg.data[1:])
        except Exception as e:
            self.get_logger().error(f"Failed to write register: {e}")
            self.is_connected = False

    def joint_state_publisher(self):
        """
        Publishes a JointState message using :attr:`current_joint_state`.  
        Also updates :attr:`joint_state_feedback` for action feedback purposes.
        """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Example 12 joint names
        joint_state_msg.name = [
            'F1M1', 'F1M2', 'F1M3', 'F1M4',
            'F2M1', 'F2M2', 'F2M3', 'F2M4',
            'F3M1', 'F3M2', 'F3M3', 'F3M4'
        ]

        joint_state_msg.position = self.current_joint_state
        self.joint_state_feedback.positions = joint_state_msg.position
        self.joint_state_pub.publish(joint_state_msg)

    def get_position(self):
        """
        Retrieves the current joint angles (in degrees) from the gripper.

        :return: A list of current joint angles in degrees.
        :rtype: list[float]
        :raises: Logs an error if the connection is lost.
        """
        if self.is_dummy:
            return self.current_joint_state

        if not self.is_connected:
            self.get_logger().error("(get_position) Connection lost")
            return
        
        return self.delto_client.get_position()

    def set_position(self, position):
        """
        Sends a command to move the gripper to the specified angles (in degrees).

        :param position: Desired joint angles in degrees
        :type position: list[float]
        :raises: Logs an error if the connection is lost or failed to set position.
        """
        if self.is_dummy:
            self.current_joint_state = position
            return
        
        if not self.is_connected:
            self.get_logger().error("Connection lost")
            return
        
        try:
            self.delto_client.set_position(position)
        except Exception as e:
            self.get_logger().error(f"(set_position) Failed to set position: {e}")
            self.is_connected = False

    def set_motion_step(self, step):
        """
        Sets the motion step (i.e., a step size or movement resolution).

        :param step: An integer specifying the step size.
        :type step: int
        """
        if self.is_dummy:
            return
        
        if not self.is_connected:
            self.get_logger().error("Connection lost")
            return
        
        try:
            self.delto_client.set_step(step)
        except Exception as e:
            self.get_logger().error(f"Failed to set motion step: {e}")
            self.is_connected = False

    def grasp_mode_callback(self, mode: Int32):
        """
        Callback function to set the gripper's grasp mode.

        :param mode: The desired grasp mode (e.g., PINCH, WIDE, etc., depending on implementation).
        :type mode: Int32
        """
        if not self.is_connected:
            self.get_logger().error("Connection lost")
            return
        
        try:
            self.delto_client.grasp_mode(mode.data)
        except Exception as e:
            self.get_logger().error(f"Failed to set grasp mode: {e}")
            self.is_connected = False

    def grasp_callback(self, grasp: Bool):
        """
        Callback function to execute a grasp operation.  
        If True, the gripper will close; if False, it will open.

        :param grasp: Whether to close (True) or open (False) the gripper.
        :type grasp: Bool
        """
        if not self.is_connected:
            self.get_logger().error("Connection lost")
            return
        
        try:
            self.delto_client.grasp(grasp.data)
        except Exception as e:
            self.get_logger().error(f"Failed to grasp: {e}")
            self.is_connected = False

    def timer_callback(self):
        """
        Periodic timer callback. If the connection is active, 
        publishes :meth:`joint_state_publisher`.
        """
        if not self.is_connected and self.reconnect_attempts <= 1:
            self.get_logger().error("Connection lost")
            return
        
        if self.is_connected:
            self.joint_state_publisher()

    def goal_callback(self, goal_request):
        """
        Callback for receiving a goal request in the FollowJointTrajectory action server.

        :param goal_request: The incoming goal request.
        :type goal_request: FollowJointTrajectory.GoalRequest
        :return: GoalResponse.ACCEPT
        """
        self.get_logger().info('Received goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Callback for receiving a cancel request in the FollowJointTrajectory action server.

        :param goal_handle: The goal handle to be canceled.
        :type goal_handle: ServerGoalHandle
        :return: CancelResponse.ACCEPT
        """
        self.get_logger().info('Received cancel')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Executes the trajectory received via the FollowJointTrajectory action server.

        :param goal_handle: Goal handle containing the trajectory.
        :type goal_handle: ServerGoalHandle
        :return: A result object indicating success/failure.
        :rtype: FollowJointTrajectory.Result
        """
        print('FollowJointTrajectory callback...')
        print(goal_handle.request.trajectory)
        goal = goal_handle.request.trajectory.points.copy()
        
        self.joint_state_list = []

        if goal:
            self.joint_state_list = [p.positions for p in goal]
        else:
            self.stop_motion()
            return

        if self.joint_state_list:
            new_array = [self.joint_state_list[0]]
            middle_point = self.joint_state_list[int(len(self.joint_state_list)/2)]
            new_array.append(middle_point)
            new_array.append(self.joint_state_list[-1])

            self.set_motion_step(1)
            new_array = [[self._rad2deg(joints) for joints in subset]
                         for subset in new_array]

            self._waypointMove(new_array, 0.5)

            result = FollowJointTrajectory.Result()
            feedback_msg = FollowJointTrajectory.Feedback()

            feedback_msg.desired.positions = self.joint_state_feedback.positions
            feedback_msg.actual.positions = self.joint_state_feedback.positions
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.01)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        print("success")
        return result

    def target_joint_callback(self, msg):
        """
        Callback for receiving the target joint angles (in radians)
        via a Float32MultiArray with 12 elements.

        :param msg: Contains 12 floats representing target joint angles in radians.
        :type msg: Float32MultiArray
        :raises: Logs an error if the message length is not 12.
        """
        if len(msg.data) != 12:
            self.get_logger().error("Invalid target joint state")
            return

        if not self.is_connected:
            self.get_logger().error("Connection lost")
            return
        
        # Convert rad -> deg
        msg.data = [self._rad2deg(x) for x in msg.data]
        self.target_joint_state = msg.data

        try:
            self.delto_client.set_position(self.target_joint_state)
        except Exception as e:
            self.get_logger().error(f"Failed to set target joint state: {e}")
            self.is_connected = False

    def fixed_joint_callback(self, msg):
        """
        Callback for receiving fixed joint states (locking certain joints) 
        via an Int16MultiArray with 12 elements.

        :param msg: Contains 12 integers specifying fixed joint values.
        :type msg: Int16MultiArray
        :raises: Logs an error if the message length is not 12.
        """
        if len(msg.data) != 12:
            self.get_logger().error("Invalid fixed joint state")
            return
    
        if not self.is_connected:
            self.get_logger().error("Connection lost")
            return
    
        self.fixed_joint_state = list(msg.data)
        print(self.fixed_joint_state)
    
        try:
            self.delto_client.fix_position(self.fixed_joint_state)
        except Exception as e:
            self.get_logger().error(f"Failed to set fixed joint state: {e}")
            self.is_connected = False

    def waypointMove(self, waypointList, threshold):
        """
        Spawns a separate thread that iterates over the list of waypoints, 
        moving the gripper to each waypoint in turn.

        :param waypointList: A list of waypoints, where each waypoint is 12 joint angles in degrees.
        :type waypointList: list[list[float]]
        :param threshold: Threshold for considering a waypoint reached (fractional error tolerance).
        :type threshold: float
        """
        self.stop_thread = False
        self.waypoint_thread = threading.Thread(
            target=self._waypointMove, args=(waypointList, threshold))
        self.waypoint_thread.start()

    def stop_motion(self):
        """
        Stops the current motion by setting the position to the current gripper pose,
        effectively halting any ongoing trajectory motion.
        """
        if not self.is_connected:
            self.get_logger().error("Connection lost")
            return
        
        self.delto_client.set_position(self.delto_client.get_position())
        self.stop_thread = True

    def _waypointMove(self, waypointList, threshold=0.3):
        """
        Internal method that iterates over the waypoint list. For each waypoint, 
        it waits until the gripper has converged within the threshold of the target 
        before moving on to the next waypoint.

        :param waypointList: A list of waypoints (each containing 12 joint angles in degrees).
        :type waypointList: list[list[float]]
        :param threshold: Fractional error tolerance (e.g., 0.3)
        :type threshold: float
        """
        self.stop_thread = False
        i = 0
        move_flag = False
        first_input_flag = True

        first_current_position = [0.0]*12
        current_position = [0.0]*12

        while not self.stop_thread:

            if not move_flag:
                self.delto_client.set_position(waypointList[i])
                move_flag = True
                first_input_flag = True

            current_position = self.delto_client.get_position()

            if first_input_flag:
                first_current_position = current_position
                first_input_flag = False

            error = []
            for a, b, c in zip(current_position, waypointList[i], first_current_position):
                if (b - c) < 0.0001:
                    error.append(0.0)
                else:
                    error.append(abs((a - b) / (b - c)))

            if all(e < threshold for e in error):
                move_flag = False
                i += 1

            if i >= len(waypointList):
                self.stop_thread = True
                return

    def _deg2rad(self, deg):
        """
        Converts an angle from degrees to radians.

        :param deg: Angle in degrees
        :type deg: float
        :return: Angle in radians
        :rtype: float
        """
        return deg * math.pi / 180.0

    def _rad2deg(self, rad):
        """
        Converts an angle from radians to degrees.

        :param rad: Angle in radians
        :type rad: float
        :return: Angle in degrees
        :rtype: float
        """
        return rad * 180.0 / math.pi


def main(args=None):
    """
    Initializes and runs the node.  
    Attempts to connect to the gripper hardware via :meth:`DeltoROSDriver.connect`, 
    then starts the ROS2 spinner.

    :param args: Command-line arguments
    :type args: list, optional
    """
    rclpy.init(args=args)

    delto_driver = DeltoROSDriver()
    connect = delto_driver.connect()

    if not connect:
        delto_driver.get_logger().error("Init network connection failed.")

    time.sleep(0.1)
    delto_driver.get_logger().info("delto_driver initialized")

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(delto_driver)
    executor.spin()
    executor.shutdown()


if __name__ == '__main__':
    main()
