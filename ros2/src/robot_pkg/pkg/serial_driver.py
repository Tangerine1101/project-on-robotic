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
        
        # Use a Reentrant group so the Action loop doesn't block the Serial Reader
        self.group = ReentrantCallbackGroup()

        # --- ARDUINO PROTOCOL ---
        # Match Arduino's config.h: NODE_STARTBYTE = 0xAA, NODE_SENDBYTE = 0xFE
        self.START_BYTE = 0xAA  # 0xAA = NODE_STARTBYTE (commands sent TO Arduino)
        self.PKG_FORMAT = '<B c B 5f B'  # StartByte, CmdID, Bitmask, 5x Float Args, Checksum
        # Incoming (Arduino -> PC) package format: SendByte, ProcessingID, StatusID, 5x Float Args, Checksum
        self.IN_PKG_FORMAT = '<B c c 5f B'
        self.IN_PKG_SIZE = struct.calcsize(self.IN_PKG_FORMAT)
        # Receive buffer for assembling partial packets
        self._rx_buffer = bytearray()

        # --- 1. LOAD PARAMETERS (From YAML) ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('tolerance', 1.0)  # Degrees
        self.declare_parameter('timeout_sec', 10.0)
        self.declare_parameter('read_frequency', 20.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.tolerance = self.get_parameter('tolerance').value
        self.timeout = self.get_parameter('timeout_sec').value
        
        # --- 2. STATE VARIABLES ---
        self.current_joints = [] # Stores [j1, j2, j3, j4, j5]
        self.serial_conn = None
        self.is_calibrated = False
        self.gripstate = False
        self.latest_ack = ""
        # --- 3. HARDWARE CONNECTION ---
        self.connect_serial()

        # --- 4. INTERFACES ---
        # Action Server (The Brain)
        self._action_server = ActionServer(
            self, MoveArm, 'move_arm', 
            execute_callback=self.execute_move_callback,
            callback_group=self.group
        )

        # Service (The Tools)
        self.srv = self.create_service(
            GripCommand, 'grip_control', 
            self.grip_handle_callback, callback_group=self.group
        )

        # Publisher (The Feedback)
        self.joint_pub = self.create_publisher(Float64MultiArray, 'joint_states', 10)
        
        # Serial Reader Timer (The Heartbeat)
        freq = self.get_parameter('read_frequency').value
        # assign read_serial_data() to a timer/callback - meaning the function will be run in spin() loop
        self.create_timer(1.0 / freq, self.read_serial_data, callback_group=self.group)

        self.get_logger().info(f"✅ Hardware Driver Online on {self.port}")

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2) # Allow Arduino reset
            self.serial_conn.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().fatal(f"❌ Could not open serial port: {e}")
            # We don't exit, just let it fail gracefully or retry logic could be added here

    # --- BINARY PACKING ---
    def send_binary_pkg(self, cmd_id: str, args: list, bitmask: int = 0b00011111):
        """
        Packs and sends the serialPackage structure.
        :param cmd_id: The command character (e.g., 'A')
        :param args: List of floats (angles)
        :param bitmask: The active motors (passed from Action or default 0x00)
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().error("Serial not connected!")
            return

        # Ensure exactly 5 arguments (pad with 0.0 if short)
        full_args = (args + [0.0]*5)[:5]
        
        # Ensure bitmask fits in signed char
        safe_bitmask = bitmask & 0xFF 

        # Checksum Logic
       
        raw_data = struct.pack('<B c B 5f', self.START_BYTE, cmd_id.encode(), int(safe_bitmask), *full_args)
        
        checksum = 0
        for b in raw_data:
            checksum ^= b
        
        # Final Pack with Checksum
        final_pkg = struct.pack(self.PKG_FORMAT, self.START_BYTE, cmd_id.encode(), int(safe_bitmask), *full_args, checksum)
        self.serial_conn.write(final_pkg)
        # Log packet send for debugging
        try:
            self.get_logger().debug(f"Sent Packet: cmd={cmd_id} mask={safe_bitmask:#04x} args={full_args}")
        except Exception:
            pass

    # --- THE CRITICAL HARDWARE LOOP (Action) ---
    async def execute_move_callback(self, goal_handle):
        self.get_logger().info(f'🚀 Moving to: {goal_handle.request.targets}')
        
        targets = goal_handle.request.targets
        
        # 1. Send Command to Arduino (forward bitmask from goal)
        # Extract bitmask from the goal (MoveArm.Goal.bitmask) if provided
        try:
            goal_bmask = int(goal_handle.request.bitmask) & 0xFF
        except Exception:
            goal_bmask = 0b00011111

        self.get_logger().debug(f"Sending move packet: targets={targets} bitmask={goal_bmask:#04x}")
        self.send_binary_pkg('A', list(targets), bitmask=goal_bmask)
        
        start_time = self.get_clock().now()
        feedback_msg = MoveArm.Feedback()
        
        # 2. Monitor Loop (Wait for arrival)
        while rclpy.ok():
            # SAFETY: Check Timeout
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.timeout:
                self.get_logger().error("⏰ Move Timed Out! Aborting.")
                goal_handle.abort()
                result = MoveArm.Result()
                result.success = False
                return result

            # WAIT: If no data from Arduino yet
            if not self.current_joints:
                time.sleep(0.1)
                continue

            # CHECK: Calculate Max Error
            # We compare only the joints we were asked to move
            # Assuming targets matches length of current_joints (or close enough)
            max_error = 0.0
            try:
                for t, c in zip(targets, self.current_joints):
                    err = abs(t - c)
                    if err > max_error: max_error = err
            except Exception as e:
                pass # Length mismatch protection

            # FEEDBACK: Tell Planner where we are
            feedback_msg.current_values = self.current_joints
            goal_handle.publish_feedback(feedback_msg)

            # SUCCESS CONDITION
            if max_error < self.tolerance:
                self.get_logger().info(f"🎯 Target Reached. Error: {max_error:.2f}")
                break

            time.sleep(0.1) # Don't choke the CPU

        goal_handle.succeed()
        result = MoveArm.Result()
        result.success = True
        return result

    # --- SERVICE ---
    def grip_handle_callback(self, request, response):
        cmd_map = {
            "open": ('R', [90.0]*5),
            "close": ('G', [90.0]*5),
            "calibrate": ('F', [90.0]*5)
        }
        
        # Map commands to the EXPECTED Arduino string
        # I corrected your logic here. Open -> Released, Close -> Griped.
        ack_map = {
            "open": "@released",
            "close": "@griped",
            "calibrate": "@Calibrated"
        }
        
        if request.command in cmd_map:
            cid, args = cmd_map[request.command]
            expected_ack = ack_map.get(request.command, "")
            
            # 1. Clear previous ack to avoid false positives
            self.latest_ack = ""
            
            # 2. Send Command
            self.send_binary_pkg(cid, args)
            
            # 3. Wait for Confirmation (Sync Block)
            # Since we use ReentrantCallbackGroup + MultiThreadedExecutor, 
            # this loop won't block the serial reader!
            start_wait = time.time()
            timeout = 3.0 # Don't wait forever, baka
            
            while (time.time() - start_wait) < timeout:
                if self.latest_ack == expected_ack:
                    self.get_logger().info(f"✅ Confirmed: {self.latest_ack}")
                    response.success = True
                    return response # Exit early on success
                time.sleep(0.1) # Yield to let the reader update
            
            # If we get here, we timed out
            self.get_logger().error(f"⏰ Service Timed Out waiting for {expected_ack}")
            response.success = False
            
        else:
            response.success = False
        
        return response

    # --- SERIAL READER (20Hz) ---
    # --- SERIAL READER (Rewritten by Gchan) ---
    def read_serial_data(self):
        """
        Reads binary packets from the MCU strictly matching 'struct sendPackage'.
        Format: <StartByte><ProcID><StatusID><Float*5><Checksum>
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        try:
            # 1. Read everything available into the buffer
            if self.serial_conn.in_waiting > 0:
                self._rx_buffer.extend(self.serial_conn.read(self.serial_conn.in_waiting))

            # 2. Sliding Window Packet Search
            # We loop as long as we have enough bytes for at least one packet
            while len(self._rx_buffer) >= self.IN_PKG_SIZE:
                
                # Check for Start Byte (Arduino uses NODE_SENDBYTE = 0xFE for outgoing packages)
                # We interpret as unsigned char to match uint8_t
                if self._rx_buffer[0] != 0xFE: # 0xFE = NODE_SENDBYTE
                    # If index 0 isn't NODE_SENDBYTE, pop it and try next byte.
                    # This effectively 'slides' the window forward.
                    self._rx_buffer.pop(0)
                    continue

                # We have a candidate start byte. Now check the Checksum.
                # Extract the candidate packet
                candidate_pkt = self._rx_buffer[:self.IN_PKG_SIZE]
                
                # Calculate XOR Checksum of [0...N-2]
                calc_checksum = 0
                for b in candidate_pkt[:-1]:
                    calc_checksum ^= b
                
                recv_checksum = candidate_pkt[-1]

                # 3. Validation
                if calc_checksum == recv_checksum:
                    # VALID PACKET FOUND
                    try:
                        # Unpack: < B c c 5f B
                        # B: StartByte
                        # c: ProcessingID (char)
                        # c: StatusID (char)
                        # 5f: Arguments
                        # B: Checksum
                        data = struct.unpack(self.IN_PKG_FORMAT, candidate_pkt)
                        
                        proc_id = data[1].decode('utf-8', errors='replace')
                        status_id = data[2].decode('utf-8', errors='replace')
                        args = list(data[3:8]) # The 5 floats

                        # Update State
                        self.current_joints = args
                        
                        # Handle Acknowledgments based on your 'operate()' logic
                        if proc_id == 'R' and status_id == 'D':
                            self.latest_ack = "@released"
                        elif proc_id == 'G' and status_id == 'D':
                            self.latest_ack = "@griped"
                        elif proc_id == 'F' and status_id == 'D':
                            self.is_calibrated = True
                            self.latest_ack = "@Calibrated"
                        
                        # Publish Joint States
                        msg = Float64MultiArray()
                        msg.data = self.current_joints
                        self.joint_pub.publish(msg)

                        # Remove the processed packet from buffer
                        del self._rx_buffer[:self.IN_PKG_SIZE]

                    except struct.error as e:
                        self.get_logger().error(f"Struct Unpack Failed: {e}")
                        # If unpack fails, it's garbage. Drop one byte and retry.
                        self._rx_buffer.pop(0)

                else:
                    # Checksum Mismatch. 
                    # This means the '@' we found was likely part of a Float, not a header.
                    # Drop the first byte and search again.
                    self.get_logger().debug(f"Checksum Fail: Calc {calc_checksum} != Recv {recv_checksum}")
                    self._rx_buffer.pop(0)
        
        except Exception as e:
            self.get_logger().error(f"Serial Loop Critical Fail: {e}")
def main(args=None):
    rclpy.init(args=args)
    # MUST use MultiThreadedExecutor for ReentrantCallbackGroup to work!
    node = SerialDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn: node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
