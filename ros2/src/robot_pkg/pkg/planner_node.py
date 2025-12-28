#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json
import numpy as np
import math
import time

# IMPORT INTERFACES
from robot_interfaces.action import MoveArm
from robot_interfaces.srv import GripCommand
from std_srvs.srv import SetBool # Fallback if needed

# IMPORT MATH
try:
    from .FWK_Degree import IK_fulls_1
except ImportError:
    from FWK_Degree import IK_fulls_1

# STATES ENUM
class State:
    INIT = 0
    SCANNING = 1
    MOVING_PICK = 2
    GRIPPING = 3
    MOVING_DROP = 4
    RELEASING = 5
    HOME = 6

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.group = ReentrantCallbackGroup()

        # --- PARAMETERS ---
        self.declare_parameter('pickup_height_mm', 50.0)
        self.declare_parameter('zone_onion_x', 150.0)
        self.declare_parameter('zone_onion_y', 150.0)
        self.declare_parameter('zone_onion_z', 100.0)
        self.declare_parameter('zone_garlic_x', -150.0)
        self.declare_parameter('zone_garlic_y', 150.0)
        self.declare_parameter('zone_garlic_z', 100.0)
        self.declare_parameter('home_angles', [0.0, 0.0, 0.0, 0.0])

        # --- CLIENTS ---
        # 1. Action Client (The Arm)
        self._action_client = ActionClient(self, MoveArm, 'move_arm', callback_group=self.group)
        
        # 2. Service Client (The Hand)
        self._grip_client = self.create_client(GripCommand, 'grip_control', callback_group=self.group)

        # 3. Vision Subscriber
        self.latest_objects = []
        self.sub_vision = self.create_subscription(
            String, '/vision/detections', self.vision_callback, 10, callback_group=self.group)

        # --- STATE MACHINE ---
        self.state = State.INIT
        self.current_target = None # Stores {'name': 'onion', 'x': ...}
        
        # The Brain Loop (Runs every 0.5s to decide what to do)
        self.timer = self.create_timer(0.5, self.brain_loop, callback_group=self.group)

        self.get_logger().info("🧠 Planner Brain Online. Waiting for system...")

    # --- CALLBACKS ---
    def vision_callback(self, msg):
        """Just updates the latest memory. Does not trigger action directly."""
        try:
            self.latest_objects = json.loads(msg.data)
        except:
            pass

    # --- THE BRAIN ---
    async def brain_loop(self):
        
        # STATE 0: INITIALIZATION & CALIBRATION
        if self.state == State.INIT:
            self.get_logger().info("⚙️ State: INIT - Requesting Calibration...")
            if not self._grip_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("Driver Service not available!")
                return
            
            # Send Calibrate Command
            req = GripCommand.Request()
            req.command = "calibrate"
            await self._grip_client.call_async(req)
            
            self.get_logger().info("✅ Calibrated. Switching to SCANNING.")
            self.state = State.SCANNING

        # STATE 1: LOOKING FOR OBJECTS
        elif self.state == State.SCANNING:
            if not self.latest_objects:
                # self.get_logger().info("... No objects seen ...")
                # Optional: Counter to switch to HOME if empty for too long
                return 

            # Sort by distance (closest first)
            self.latest_objects.sort(key=lambda obj: math.hypot(obj['x'], obj['y']))
            target = self.latest_objects[0] # Pick closest
            
            self.get_logger().info(f"👀 Found {target['name']} at ({target['x']}, {target['y']})")
            
            # Validate Reachability BEFORE changing state
            angles = self.solve_ik(target['x']*10, target['y']*10, self.get_parameter('pickup_height_mm').value)
            
            if angles is not None:
                self.current_target = target
                self.target_angles = angles
                self.state = State.MOVING_PICK
            else:
                self.get_logger().warn("⚠️ Object out of reach! Ignoring.")

        # STATE 2: MOVING TO PICK
        elif self.state == State.MOVING_PICK:
            self.get_logger().info("🚀 State: MOVING_PICK")
            success = await self.send_arm_goal(self.target_angles)
            
            if success:
                self.state = State.GRIPPING
            else:
                self.get_logger().error("Failed to reach object. Retrying SCAN.")
                self.state = State.SCANNING

        # STATE 3: GRIPPING
        elif self.state == State.GRIPPING:
            self.get_logger().info("✊ State: GRIPPING")
            req = GripCommand.Request()
            req.command = "close"
            await self._grip_client.call_async(req)
            time.sleep(1.0) # Wait for physical grip
            self.state = State.MOVING_DROP

        # STATE 4: MOVING TO CLASSIFICATION ZONE
        elif self.state == State.MOVING_DROP:
            self.get_logger().info(f"🚚 State: MOVING_DROP ({self.current_target['name']})")
            
            # 1. Determine Zone
            name = self.current_target['name'].lower()
            if "onion" in name:
                zx = self.get_parameter('zone_onion_x').value
                zy = self.get_parameter('zone_onion_y').value
                zz = self.get_parameter('zone_onion_z').value
            elif "garlic" in name:
                zx = self.get_parameter('zone_garlic_x').value
                zy = self.get_parameter('zone_garlic_y').value
                zz = self.get_parameter('zone_garlic_z').value
            else:
                # Default unknown bin
                zx, zy, zz = 150.0, 0.0, 100.0

            # 2. Solve IK for Zone
            angles = self.solve_ik(zx, zy, zz)
            
            if angles is not None:
                await self.send_arm_goal(angles)
                self.state = State.RELEASING
            else:
                self.get_logger().error("❌ Zone unreachable! Dropping here.")
                self.state = State.RELEASING

        # STATE 5: RELEASING
        elif self.state == State.RELEASING:
            self.get_logger().info("🖐 State: RELEASING")
            req = GripCommand.Request()
            req.command = "open"
            await self._grip_client.call_async(req)
            time.sleep(1.0)
            
            # Check if more objects exist
            if self.latest_objects:
                self.state = State.SCANNING
            else:
                self.state = State.HOME

        # STATE 6: HOME & SHUTDOWN
        elif self.state == State.HOME:
            self.get_logger().info("🏠 State: HOME (Mission Complete)")
            
            # 1. Move to Home Angles (Direct angle control, no IK needed usually)
            home = self.get_parameter('home_angles').value
            await self.send_arm_goal(home)
            
            # 2. Close Grip
            req = GripCommand.Request()
            req.command = "close"
            await self._grip_client.call_async(req)
            
            self.get_logger().info("💤 Robot Sleeping. Waiting for new objects...")
            self.state = State.SCANNING # Loop back to check for new stuff

    # --- HELPER: SEND ACTION ---
    async def send_arm_goal(self, angles):
        goal = MoveArm.Goal()
        goal.targets = [float(a) for a in angles]
        goal.bitmask = 0x1F # Enable all motors

        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal)
        
        # Wait for acceptance
        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        # Wait for result
        res_future = goal_handle.get_result_async()
        result = await res_future
        return result.result.success

    # --- HELPER: MATH DEPT ---
    def solve_ik(self, x_mm, y_mm, z_mm):
        # Construct Matrix T
        # T definition depends on your specific FWK logic
        # Assuming simple translation identity for now
        T = np.array([
            [1.0, 0.0, 0.0, x_mm],
            [0.0, -1.0, 0.0, y_mm], 
            [0.0, 0.0, -1.0, z_mm], 
            [0.0, 0.0, 0.0, 1.0]
        ])
        
        # Call imported IK function
        try:
            solutions = IK_fulls_1(T)
            if len(solutions) > 0:
                # Return the first valid solution
                return solutions[0]
        except Exception as e:
            self.get_logger().error(f"IK Math Error: {e}")
        
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()