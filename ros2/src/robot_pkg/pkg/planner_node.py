#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import time
import numpy as np

# --- IMPORTS ---
from robot_interfaces.action import MoveArm
from robot_interfaces.srv import GripCommand, MoveTo
from robot_interfaces.msg import ObjectList

# Math Import
try:
    from .FWK_Degree import IK_fulls_1
except ImportError:
    try:
        from FWK_Degree import IK_fulls_1
    except Exception as e:
        print(f"CRITICAL ERROR: Cannot import IK library: {e}")

class State:
    WAIT_DEPS = 0    
    CALIBRATING = 1  
    MOVING_HOME = 2
    PRE_TASK_OPEN = 3 
    SCANNING = 4     
    MOVING_PICK = 5  
    GRIPPING = 6
    MOVING_CENTER = 7 
    MOVING_DROP = 8  
    RELEASING = 9    
    MANUAL = 99      

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.group = ReentrantCallbackGroup()

        # --- PARAMETERS ---
        self.declare_parameter('wait_vision', True)
        self.declare_parameter('manual_mode', False)
        
        self.declare_parameter('pickup_height_mm', 0.0)
        self.declare_parameter('home_angles', [0.0, 40.0, -10.0, 0.0])
        
        # Zones
        self.declare_parameter('zone_onion_x', 320.0)
        self.declare_parameter('zone_onion_y', -250.0)
        self.declare_parameter('zone_garlic_x', 180.0)
        self.declare_parameter('zone_garlic_y', -250.0)
        self.declare_parameter('zone_lemon_x', 250.0)
        self.declare_parameter('zone_lemon_y', -250.0)

        # Exclusion Radius (Simpler than Rectangles)
        self.declare_parameter('zone_radius_mm', 150.0) # 15cm Radius

        # --- CLIENTS ---
        self._action_client = ActionClient(self, MoveArm, 'move_arm', callback_group=self.group)
        self._grip_client = self.create_client(GripCommand, 'grip_control', callback_group=self.group)

        # --- SERVICES ---
        self._move_srv = self.create_service(MoveTo, 'move_to_position', self.move_to_callback, callback_group=self.group)

        # --- VISION ---
        self.latest_objects = []
        self.last_vision_time = 0.0
        self.vision_active = False
        self.sub_vision = self.create_subscription(
            ObjectList, '/vision/detections', self.vision_callback, 10, callback_group=self.group)

        # --- STATE MANAGEMENT ---
        manual_flag = self.get_parameter('manual_mode').value
        self.state = State.MANUAL if manual_flag else State.WAIT_DEPS
        
        self.calibration_sent = False 
        self.current_target = None
        self.target_angles = []
        self.busy = False 

        self.timer = self.create_timer(0.5, self.brain_loop, callback_group=self.group)
        
        mode_str = "MANUAL" if manual_flag else "AUTO"
        self.get_logger().info(f"🧠 Planner Brain Online. Mode: {mode_str}")

    def vision_callback(self, msg):
        self.latest_objects = msg.objects
        self.last_vision_time = time.time()
        self.vision_active = True

    async def move_to_callback(self, request, response):
        self.get_logger().info(f"🔧 Manual Command: Go to ({request.x}, {request.y}, {request.z})")
        self.state = State.MANUAL 
        angles = self.solve_ik(request.x, request.y, request.z)
        if angles is not None:
            success = await self.send_arm_goal(angles)
            response.success = success
        else:
            response.success = False
            self.get_logger().error("⚠️ Manual Move Failed (IK Unreachable).")
        return response

    # --- THE BRAIN ---
    async def brain_loop(self):
        if self.busy: return 
        if self.state == State.MANUAL: return

        self.busy = True 
        try:
            # 1. WAIT FOR DEPENDENCIES
            if self.state == State.WAIT_DEPS:
                if not self._grip_client.service_is_ready():
                    self.get_logger().warn("⏳ Waiting for Driver...", throttle_duration_sec=2)
                else:
                    wait_vis = self.get_parameter('wait_vision').value
                    is_stale = (time.time() - self.last_vision_time) > 2.0
                    
                    if wait_vis and (not self.vision_active or is_stale):
                        self.get_logger().warn("⏳ Waiting for Vision...", throttle_duration_sec=2)
                    else:
                        self.get_logger().info("✅ Ready. Requesting Calibration.")
                        self.state = State.CALIBRATING

            # 2. CALIBRATION
            elif self.state == State.CALIBRATING:
                if not self.calibration_sent:
                    self.calibration_sent = True 
                    req = GripCommand.Request()
                    req.command = "calibrate"
                    self.get_logger().info("🛠️ Calibrating...")
                    result = await self._grip_client.call_async(req)
                    if result.success:
                        self.get_logger().info("✅ Calibrated.")
                        self.state = State.MOVING_HOME
                    else:
                        self.get_logger().error("❌ Calibration Failed! Retrying...")
                        self.calibration_sent = False 

            # 3. MOVE HOME
            elif self.state == State.MOVING_HOME:
                self.get_logger().info("🏠 Moving Home...")
                home_angles = self.get_parameter('home_angles').value
                success = await self.send_arm_goal(home_angles)
                if success:
                    self.state = State.PRE_TASK_OPEN
                else:
                    self.get_logger().error("❌ Home Move Failed.")

            # 4. PRE-TASK OPEN
            elif self.state == State.PRE_TASK_OPEN:
                req = GripCommand.Request()
                req.command = "open"
                await self._grip_client.call_async(req)
                self.latest_objects = [] 
                self.state = State.SCANNING

            # 5. SCANNING
            elif self.state == State.SCANNING:
                if not self.latest_objects:
                    self.get_logger().info("👀 Scanning...", throttle_duration_sec=2)
                else:
                    valid_names = ['onion', 'garlic', 'lemon']
                    candidates = []
                    
                    # Debug Info
                    self.get_logger().info(f"👀 Analyzing {len(self.latest_objects)} objects...")

                    for obj in self.latest_objects:
                        if obj.name.lower() not in valid_names: continue
                        
                        obj_x_mm = obj.x * 10.0
                        obj_y_mm = obj.y * 10.0
                        
                        # Use Radius Check
                        in_zone, dist, nearest_zone = self.check_zone_radius(obj_x_mm, obj_y_mm)
                        
                        if in_zone:
                            self.get_logger().info(f"🚫 Ignoring {obj.name} at ({obj_x_mm:.0f}, {obj_y_mm:.0f}). Inside {nearest_zone} (Dist: {dist:.0f}mm)")
                            continue
                        else:
                            # Log why it's accepted
                            self.get_logger().info(f"✅ Accepting {obj.name} at ({obj_x_mm:.0f}, {obj_y_mm:.0f}). Dist to {nearest_zone}: {dist:.0f}mm")

                        candidates.append(obj)
                    
                    if not candidates:
                        self.get_logger().info("👀 All objects filtered.", throttle_duration_sec=2)
                    else:
                        closest_obj = min(candidates, key=lambda o: math.hypot(o.x, o.y))
                        self.get_logger().info(f"🎯 Target Acquired: {closest_obj.name}")
                        pickup_z = self.get_parameter('pickup_height_mm').value
                        angles = self.solve_ik(closest_obj.x * 10, closest_obj.y * 10, pickup_z)
                        if angles is not None:
                            self.current_target = closest_obj
                            self.target_angles = angles
                            self.state = State.MOVING_PICK
                        else:
                            self.get_logger().warn("⚠️ Unreachable.")
                            self.latest_objects = []

            # 6. MOVING PICK
            elif self.state == State.MOVING_PICK:
                self.get_logger().info("🚀 Picking...")
                success = await self.send_arm_goal(self.target_angles)
                self.state = State.GRIPPING if success else State.MOVING_HOME

            # 7. GRIPPING
            elif self.state == State.GRIPPING:
                self.get_logger().info("✊ Gripping...")
                req = GripCommand.Request()
                req.command = "close"
                await self._grip_client.call_async(req)
                self.state = State.MOVING_CENTER

            # 8. MOVING CENTER
            elif self.state == State.MOVING_CENTER:
                self.get_logger().info("📍 Lifting [0,0,0,0]...")
                success = await self.send_arm_goal([0.0, 0.0, 0.0, 0.0])
                self.state = State.MOVING_DROP

            # 9. MOVING DROP
            elif self.state == State.MOVING_DROP:
                name = self.current_target.name.lower()
                self.get_logger().info(f"🚚 Dropping {name}...")
                
                if 'onion' in name:
                    zx = self.get_parameter('zone_onion_x').value
                    zy = self.get_parameter('zone_onion_y').value
                elif 'garlic' in name:
                    zx = self.get_parameter('zone_garlic_x').value
                    zy = self.get_parameter('zone_garlic_y').value
                elif 'lemon' in name:
                    zx = self.get_parameter('zone_lemon_x').value
                    zy = self.get_parameter('zone_lemon_y').value
                else:
                    zx, zy = 150.0, 150.0

                angles = self.solve_ik(zx, zy, 100.0)
                if angles is not None:
                    await self.send_arm_goal(angles)
                
                self.state = State.RELEASING

            # 10. RELEASING
            elif self.state == State.RELEASING:
                req = GripCommand.Request()
                req.command = "open"
                await self._grip_client.call_async(req)
                self.get_logger().info("✅ Done.")
                self.state = State.MOVING_HOME

        finally:
            self.busy = False

    # --- HELPERS ---
    def check_zone_radius(self, x_mm, y_mm):
        """Returns (in_zone, distance_to_center, zone_name)"""
        zones = {
            'OnionZone': (self.get_parameter('zone_onion_x').value, self.get_parameter('zone_onion_y').value),
            'GarlicZone': (self.get_parameter('zone_garlic_x').value, self.get_parameter('zone_garlic_y').value),
            'LemonZone': (self.get_parameter('zone_lemon_x').value, self.get_parameter('zone_lemon_y').value)
        }
        radius = self.get_parameter('zone_radius_mm').value
        
        nearest_dist = float('inf')
        nearest_name = "None"
        
        for name, (zx, zy) in zones.items():
            dist = math.hypot(x_mm - zx, y_mm - zy)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_name = name
            
            if dist <= radius:
                return True, dist, name
        
        return False, nearest_dist, nearest_name

    async def send_arm_goal(self, angles):
        goal = MoveArm.Goal()
        raw = [float(a) for a in angles]
        if len(raw) > 3: raw[3] += 90.0
        goal.targets = raw
        goal.bitmask = 0x1F
        self._action_client.wait_for_server()
        goal_handle = await self._action_client.send_goal_async(goal)
        if not goal_handle.accepted: return False
        res = await goal_handle.get_result_async()
        return res.result.success

    def solve_ik(self, x, y, z):
        T = np.array([
            [1.0, 0.0, 0.0, float(x)],
            [0.0, -1.0, 0.0, float(y)], 
            [0.0, 0.0, -1.0, float(z)], 
            [0.0, 0.0, 0.0, 1.0]
        ])
        try:
            solutions = IK_fulls_1(T)
            if len(solutions) > 0: return solutions[0] 
        except: pass
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()