#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import serial
import struct
import time
import math

# Custom interface imports
from robot_interfaces.action import MoveArm
from robot_interfaces.srv import GripCommand
from std_msgs.msg import Float64MultiArray

class SerialDriver(Node):
    def __init__(self):
        super().__init__('serial_driver')
        self.group = ReentrantCallbackGroup()

        # --- PROTOCOL ---
        self.START_BYTE = 0xAA 
        self.PKG_FORMAT = '<B c B 5f B'  
        self.IN_PKG_FORMAT = '<B c c 5f B'
        self.IN_PKG_SIZE = struct.calcsize(self.IN_PKG_FORMAT)
        self._rx_buffer = bytearray()

        # --- PARAMS ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('tolerance', 2.0)
        self.declare_parameter('timeout_sec', 15.0)
        self.declare_parameter('read_frequency', 30.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.tolerance = self.get_parameter('tolerance').value
        self.timeout = self.get_parameter('timeout_sec').value
        
        # --- STATE ---
        self.current_joints = [0.0]*5 
        self.is_calibrated = False 
        self.latest_ack = ""
        self.serial_conn = None

        self.connect_serial()

        # --- INTERFACES ---
        self._action_server = ActionServer(
            self, MoveArm, 'move_arm', 
            execute_callback=self.execute_move_callback,
            callback_group=self.group
        )

        self.srv = self.create_service(
            GripCommand, 'grip_control', 
            self.grip_handle_callback, callback_group=self.group
        )

        self.joint_pub = self.create_publisher(Float64MultiArray, 'joint_states', 10)
        
        freq = self.get_parameter('read_frequency').value
        self.create_timer(1.0 / freq, self.read_serial_data, callback_group=self.group)

        self.get_logger().info(f"✅ Driver Online. Waiting for Calibration...")

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=0.1)
            self.serial_conn.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().fatal(f"❌ Serial Port Error: {e}")

    def send_binary_pkg(self, cmd_id: str, args: list, bitmask: int = 0b00011111):
        if not self.serial_conn or not self.serial_conn.is_open: return

        # PAD TO 5 FLOATS
        full_args = (args + [0.0]*5)[:5]
        safe_bitmask = bitmask & 0xFF 
        
        raw_data = struct.pack('<B c B 5f', self.START_BYTE, cmd_id.encode(), int(safe_bitmask), *full_args)
        checksum = 0
        for b in raw_data: checksum ^= b
        
        final_pkg = struct.pack(self.PKG_FORMAT, self.START_BYTE, cmd_id.encode(), int(safe_bitmask), *full_args, checksum)
        self.serial_conn.write(final_pkg)

    async def execute_move_callback(self, goal_handle):
        if not self.is_calibrated:
            self.get_logger().warn("⚠️ REJECTING MOVE: Not calibrated.")
            goal_handle.abort()
            return MoveArm.Result(success=False)

        raw_targets = goal_handle.request.targets
        
        # --- CRITICAL FIX: STRIP ACTUATOR DATA ---
        # Ensure we only send the first 4 joints via 'A' command.
        # The 5th float (Actuator) will be padded to 0.0 by send_binary_pkg,
        # ensuring 'A' command doesn't try to position the gripper.
        targets = list(raw_targets)[:4] 
        
        self.get_logger().info(f'🚀 Moving: {targets} (Actuator Ignored in Move)')
        
        # Send Command
        self.send_binary_pkg('A', targets, bitmask=0x1F)
        
        start_time = self.get_clock().now()
        feedback_msg = MoveArm.Feedback()
        
        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.timeout:
                self.get_logger().error(f"⏰ Timed Out. Current: {self.current_joints}")
                goal_handle.abort()
                return MoveArm.Result(success=False)

            max_error = 0.0
            try:
                for i, (t, c) in enumerate(zip(targets, self.current_joints)):
                    if i == 3: continue # Ignore Joint 4 check
                    err = abs(t - c)
                    if err > max_error: max_error = err
            except: pass

            feedback_msg.current_values = self.current_joints
            goal_handle.publish_feedback(feedback_msg)

            if max_error < self.tolerance:
                self.get_logger().info(f"🎯 Target Reached. Error: {max_error:.2f}")
                break
            
            time.sleep(0.05) 

        goal_handle.succeed()
        return MoveArm.Result(success=True)

    def grip_handle_callback(self, request, response):
        cmd_map = {
            "open": ('R', [90.0]*5),
            "close": ('G', [90.0]*5),
            "calibrate": ('F', [90.0]*5)
        }
        
        if request.command not in cmd_map:
            response.success = False
            return response

        cid, args = cmd_map[request.command]
        self.get_logger().info(f"🔧 Service Command: {request.command}")
        
        self.latest_ack = ""
        self.send_binary_pkg(cid, args)
        
        start = time.time()
        while (time.time() - start) < 5.0:
            if request.command == "calibrate" and self.is_calibrated:
                self.get_logger().info("✅ Calibration Confirmed!")
                response.success = True
                return response
            
            if self.latest_ack:
                self.get_logger().info(f"✅ Ack Received: {self.latest_ack}")
                response.success = True
                return response
            time.sleep(0.1)
            
        self.get_logger().error("⏰ Service Timeout")
        response.success = False
        return response

    def read_serial_data(self):
        if not self.serial_conn or not self.serial_conn.is_open: return

        try:
            if self.serial_conn.in_waiting > 0:
                self._rx_buffer.extend(self.serial_conn.read(self.serial_conn.in_waiting))

            while len(self._rx_buffer) >= self.IN_PKG_SIZE:
                if self._rx_buffer[0] != 0xFE:
                    self._rx_buffer.pop(0)
                    continue

                pkt = self._rx_buffer[:self.IN_PKG_SIZE]
                calc_sum = 0
                for b in pkt[:-1]: calc_sum ^= b
                
                if calc_sum == pkt[-1]:
                    data = struct.unpack(self.IN_PKG_FORMAT, pkt)
                    proc_id = data[1].decode('utf-8', errors='replace')
                    status_id = data[2].decode('utf-8', errors='replace')
                    self.current_joints = list(data[3:8])

                    if proc_id == 'F' and status_id == 'D':
                        self.is_calibrated = True
                        self.latest_ack = "calibrated"
                    elif status_id == 'D':
                        self.latest_ack = "done"

                    self.joint_pub.publish(Float64MultiArray(data=self.current_joints))
                    del self._rx_buffer[:self.IN_PKG_SIZE]
                else:
                    self._rx_buffer.pop(0)

        except Exception as e:
            self.get_logger().error(f"Serial Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()