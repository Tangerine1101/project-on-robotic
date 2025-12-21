#!/usr/bin/env python3

# --- IMPORTS ---
import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Float64MultiArray
from typing import Optional

# --- CLASS DEFINITION ---
class SerialDriver(Node):

    # __init__ is the Constructor. It runs AUTOMATICALLY when you create an object from this class.
    def __init__(self):
        super().__init__('serial_driver')
        
        # --- PARAMETERS ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # --- INITIAL STATE ---
        self.serial_conn: Optional[serial.Serial] = None

        # --- SERIAL CONNECTION ---
        self.try_open_serial()
        self.reconnect_timer = self.create_timer(5.0, self.try_open_serial)

        # --- SUBSCRIBER ---
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'arm_commands',
            self.listener_callback,
            10)
        self.subscription  

    # attempt to open serial if it's not already open
    def try_open_serial(self) -> None:
        port_name = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        if self.serial_conn and getattr(self.serial_conn, 'is_open', False):
            return
        try:
            self.serial_conn = serial.Serial(port_name, baud, timeout=1)
            self.get_logger().info(f'Connected to serial port: {port_name} at {baud}')
        except serial.SerialException as e:
            self.get_logger().warning(f'Unable to open {port_name}: {e}')
            self.serial_conn = None

    # --- CALLBACK FUNCTION ---
    def listener_callback(self, msg):
        # ensure serial is ready
        if not self.serial_conn or not getattr(self.serial_conn, 'is_open', False):
            self.get_logger().warning('No serial connection; dropping command.')
            return

        if len(msg.data) != 4:
            self.get_logger().error(f'Expected 4 angles, got {len(msg.data)}.')
            return

        a, b, c, d = msg.data
        command_str = f"moveto -a {a:.2f} -b {b:.2f} -c {c:.2f} -d {d:.2f}\n"
        try:
            self.serial_conn.write(command_str.encode('utf-8'))
            self.get_logger().info(f'Sent: {command_str.strip()}')
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

# --- ENTRY POINT ---
def main(args=None):
    rclpy.init(args=args)
    node = SerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()