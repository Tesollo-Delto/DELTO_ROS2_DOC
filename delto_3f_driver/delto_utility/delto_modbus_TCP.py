"""
Communication Module
====================

This module provides a class for communicating with the Delto 3F Gripper using
Modbus TCP. The class includes methods for connecting, disconnecting, reading
and writing registers, setting the gripper's position, and handling various
configuration tasks such as IP, subnet mask, and gateway settings.

Example:
    An example of connecting to the gripper, changing its IP, and then writing
    to ROM so the change persists:

    .. code-block:: python

        from delto_utility import Communication

        comm = Communication()
        comm.connect('169.254.186.72', 502)
        comm.set_ip('169.254.186.73')
        comm.rom_write()

    Note that the new IP will only apply after the power is cycled (the device is restarted).
"""

import sys
import os
import threading
import struct
import rclpy

from pymodbus.client.tcp import ModbusTcpClient
from delto_utility.delto_3f_enum import (
    Delto3F,
    Delto3FCoils,
    Delto3FHoldingRegisters,
    Delto3FInputRegisters
)

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class Communication:
    """
    Handles sending commands and receiving data from the Delto Gripper using Modbus TCP.

    :ivar client: An instance of a Modbus TCP client used for communication.
    :vartype client: ModbusTcpClient

    :ivar slaveID: The slave ID of the Delto gripper (default is 1).
    :vartype slaveID: int

    :ivar dummy: Whether the communication should be in dummy mode (i.e., no actual hardware communication).
    :vartype dummy: bool

    :ivar lock: A threading lock to ensure thread-safe register read/write operations.
    :vartype lock: threading.Lock
    """

    def __init__(self, dummy: bool = False):
        """
        Initializes the Communication class.

        :param dummy: Whether to use dummy mode (no hardware I/O).
        :type dummy: bool, optional
        """
        self.client = None
        self.slaveID = 0
        self.dummy = dummy
        self.lock = threading.Lock()

    def __del__(self):
        """
        Destructor that attempts to disconnect from the Delto gripper on object deletion.
        """
        self.disconnect()

    def write_registers(self, address: int, values: list[int]) -> None:
        """
        Writes multiple registers starting from the specified address.

        :param address: Starting register address.
        :type address: int
        :param values: List of integer values to write.
        :type values: list[int]
        """
        self.client.write_registers(
            address=address, values=values, slave=self.slaveID
        )

    def connect(self, ip: str, port: int, slaveID: int = 1) -> bool:
        """
        Connects to the Delto Gripper via Modbus TCP.

        :param ip: The IP address of the gripper.
        :type ip: str
        :param port: The TCP port for Modbus communication.
        :type port: int
        :param slaveID: The Modbus slave ID of the gripper (default=1).
        :type slaveID: int, optional
        :return: True if the connection succeeds, False otherwise.
        :rtype: bool
        """
        self.slaveID = slaveID
        if self.dummy:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name
            )
            return True  # In dummy mode, pretend success

        self.client = ModbusTcpClient(host=ip, port=port)
        return self.client.connect()

    def disconnect(self) -> None:
        """
        Disconnects from the Delto Gripper.

        Note:
            In the current implementation, the client's `close()` call
            is commented out. If you decide to use an actual close
            operation, uncomment the relevant line.
        """
        if self.dummy:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name
            )
            return

        # self.client.close()

    def get_position(self) -> list[float]:
        """
        Reads the current positions of the motors from the Delto Gripper.

        :return: A list of 12 motor positions in degrees, where each position
                 is a float. If `dummy` is True, returns a list of zeros.
        :rtype: list[float]
        """
        if self.dummy:
            status = [0]*12
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name
            )
            return status

        status = self.client.read_input_registers(
            address=Delto3FInputRegisters.MOTOR1_CURRENT_POSITION.value,
            count=Delto3F.MOTOR_NUM.value,
            slave=self.slaveID
        ).registers

        for i in range(Delto3F.MOTOR_NUM.value):
            # Convert to signed 16-bit, then scale by 1/10
            status[i] = (status[i] if status[i] < 32768 else status[i] - 65536) / 10.0

        return status

    def get_high_force(self) -> int:
        """
        Reads the high force limit from the Delto Gripper.

        :return: The high force limit as an integer.
        :rtype: int
        """
        with self.lock:
            high_force = self.client.read_input_registers(
                address=Delto3FInputRegisters.HIGH_FORCE.value,
                count=1,
                slave=self.slaveID
            ).registers
        return high_force[0]

    def get_low_force(self) -> int:
        """
        Reads the low force limit from the Delto Gripper.

        :return: The low force limit as an integer.
        :rtype: int
        """
        with self.lock:
            low_force = self.client.read_input_registers(
                address=Delto3FInputRegisters.LOW_FORCE.value,
                count=1,
                slave=self.slaveID
            ).registers
        return low_force[0]

    def set_position(self, position: list[float]) -> None:
        """
        Sends a command to set the target positions for all 12 motors.

        :param position: A list of 12 floats representing the target positions in degrees.
        :type position: list[float]
        """
        if self.dummy:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name
            )
            return

        if len(position) != 12:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name +
                " position size is not 12"
            )
            return

        with self.lock:
            # Convert float degrees to signed 16-bit integer scaled by 10
            intPositions = [
                struct.unpack('H', struct.pack('h', int(x * 10)))[0]
                for x in position
            ]
            self.client.write_registers(
                address=72, values=intPositions, slave=self.slaveID
            )

    def get_pgain(self) -> list[int]:
        """
        Reads the P gains for the 12 motors.

        :return: A list of 12 integers representing the P gains.
        :rtype: list[int]
        """
        if self.dummy:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name
            )
            return []

        with self.lock:
            pGain = self.client.read_holding_registers(
                address=Delto3FHoldingRegisters.MOTOR1_PGAIN.value,
                count=Delto3F.MOTOR_NUM.value,
                slave=self.slaveID
            ).registers
        return pGain

    def set_pgain(self, pGain: list[int]) -> None:
        """
        Writes P gains for the 12 motors.

        :param pGain: List of 12 integers representing new P gains.
        :type pGain: list[int]
        """
        # print("setPGain", pGain)
        pGain = list(pGain)
        print("setPGain", pGain)
        self.client.write_registers(
            address=Delto3FHoldingRegisters.MOTOR1_PGAIN.value,
            values=pGain,
            slave=self.slaveID
        )

    def get_dgain(self) -> list[int]:
        """
        Reads the D gains for the 12 motors.

        :return: A list of 12 integers representing the D gains.
        :rtype: list[int]
        """
        if self.dummy:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name
            )
            return []

        with self.lock:
            dGain = self.client.read_holding_registers(
                address=Delto3FHoldingRegisters.MOTOR1_DGAIN.value,
                count=Delto3F.MOTOR_NUM.value,
                slave=self.slaveID
            ).registers
        return dGain

    def set_dgain(self, dGain: list[int]) -> None:
        """
        Writes D gains for the 12 motors.

        :param dGain: List of 12 integers representing new D gains.
        :type dGain: list[int]
        """
        self.client.write_registers(
            address=Delto3FHoldingRegisters.MOTOR1_DGAIN.value,
            values=dGain,
            slave=self.slaveID
        )

    def set_free(self, isFree: bool) -> None:
        """
        Enables or disables the custom/free mode for the gripper joints.

        :param isFree: If True, sets the gripper joints to free mode.
        :type isFree: bool
        """
        self.client.write_coil(
            address=Delto3FCoils.JOINT_CUSTOM_MODE.value,
            value=isFree,
            slave=self.slaveID
        )

    def grasp_mode(self, mode: int) -> None:
        """
        Sets the gripper's grasp mode.

        :param mode: The desired grasp mode (0 = open, any other = set mode + grasp).
        :type mode: int
        """
        if mode == 0:
            self.grasp(False)
        else:
            self.client.write_register(
                address=Delto3FHoldingRegisters.GRASP_MODE.value,
                value=mode,
                slave=self.slaveID
            )
            self.grasp(True)

    def get_grasp_mode(self) -> list[int]:
        """
        Reads the gripper's current grasp mode.

        :return: A list of register values (should be 1 element with the current mode).
        :rtype: list[int]
        """
        with self.lock:
            mode = self.client.read_input_registers(
                address=Delto3FHoldingRegisters.GRASP_MODE.value,
                count=1,
                slave=self.slaveID
            ).registers
        return mode

    def grasp(self, isGrasp: bool) -> None:
        """
        Sends a command to open or close the gripper.

        :param isGrasp: True to close the gripper, False to open it.
        :type isGrasp: bool
        """
        with self.lock:
            print("Grasp", isGrasp)
            self.client.write_coil(
                address=Delto3FCoils.GRASP.value,
                value=isGrasp,
                slave=self.slaveID
            )

    def set_step(self, step: int) -> None:
        """
        Sets the motion step value. This step size influences the motion resolution.

        :param step: Motion step size. Valid range is [1, 32767].
        :type step: int
        """
        if step < 1:
            step = 1
        elif step > 32767:
            step = 32767

        self.client.write_register(
            address=Delto3FHoldingRegisters.MOTION_STEP.value,
            value=step,
            slave=self.slaveID
        )

    def rom_write(self) -> None:
        """
        Triggers a write to the EEPROM, storing any recently changed parameters
        such as IP address or gains. After this operation, a power cycle is
        typically required to apply changes (e.g. changed IP address).
        """
        self.client.write_coil(
            address=Delto3FCoils.EEPROM_WRITE.value,
            value=True,
            slave=self.slaveID
        )

    def set_ip(self, ip: str) -> None:
        """
        Sets a new IP address for the Delto Gripper.

        :param ip: The new IP address in dot-decimal format (e.g. "192.168.1.10").
        :type ip: str
        :raises ValueError: If the IP string does not split into 4 parts.
        """
        if self.dummy:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name
            )
            return

        octets = ip.split('.')
        if len(octets) != 4:
            rclpy.Node.get_logger().error(
                rclpy.Node.get_name() + ": " + "Invalid IP address"
            )
            return

        octets = list(map(int, octets))

        self.client.write_registers(
            address=Delto3FHoldingRegisters.ETHERNET_IP_CLASS_A.value,
            values=octets,
            slave=self.slaveID
        )

    def set_subnet_mask(self, subnet_mask: str) -> None:
        """
        Sets a new subnet mask for the Delto Gripper.

        :param subnet_mask: The new subnet mask in dot-decimal format (e.g. "255.255.255.0").
        :type subnet_mask: str
        :raises ValueError: If the subnet mask does not split into 4 parts.
        """
        if self.dummy:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name
            )
            return

        octets = subnet_mask.split('.')
        if len(octets) != 4:
            rclpy.Node.get_logger().error(
                rclpy.Node.get_name() + ": " + "Invalid subnet mask"
            )
            return

        octets = list(map(int, octets))
        self.client.write_registers(
            Delto3FHoldingRegisters.ETHERNET_SUBNET_MASK_A.value,
            values=octets,
            slave=self.slaveID
        )

    def set_gate_way(self, gateway: str) -> None:
        """
        Sets a new gateway IP address for the Delto Gripper.

        :param gateway: The new gateway IP in dot-decimal format (e.g. "192.168.1.1").
        :type gateway: str
        :raises ValueError: If the gateway string does not split into 4 parts.
        """
        if self.dummy:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name
            )
            return

        octets = gateway.split('.')
        if len(octets) != 4:
            rclpy.Node.get_logger().error(
                rclpy.Node.get_name() + ": " + "Invalid gateway"
            )
            return

        octets = list(map(int, octets))
        self.client.write_registers(
            Delto3FHoldingRegisters.ETHERNET_GATEWAY_A.value,
            values=octets,
            slave=self.slaveID
        )

    def fix_position(self, position: list[int]) -> None:
        """
        Locks or frees each joint position. This feature is available
        from firmware version 1.5 or higher.

        :param position: A list of 12 integers, each either 0 (free) or 1 (fixed).
        :type position: list[int]
        """
        if self.dummy:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name
            )
            return

        if len(position) != 12:
            rclpy.Node.get_logger().info(
                rclpy.Node.get_name() + ": " + sys._getframe().f_code.co_name +
                " position size is not 12"
            )
            return

        with self.lock:
            self.client.write_registers(
                address=Delto3FHoldingRegisters.MOTOR1_FIXED_POSITION.value,
                values=position,
                slave=self.slaveID
            )

    def load_pose(self, pose_index: int) -> None:
        """
        Loads a previously saved pose from internal memory.

        :param pose_index: The pose index (1~30).
        :type pose_index: int
        :raises ValueError: If the firmware version is below 1.5 or if the index is out of range.
        """
        with self.lock:
            self.client.write_register(
                address=Delto3FHoldingRegisters.MOTION_LOAD.value,
                value=pose_index,
                slave=self.slaveID
            )

    def save_pose(self, pose_index: int) -> None:
        """
        Saves the current gripper pose to internal memory.

        :param pose_index: The pose index (1~30).
        :type pose_index: int
        :raises ValueError: If the firmware version is below 1.5 or if the index is out of range.
        """
        with self.lock:
            self.client.write_register(
                address=Delto3FHoldingRegisters.SAVE_TARGET_POSE.value,
                value=pose_index,
                slave=self.slaveID
            )

