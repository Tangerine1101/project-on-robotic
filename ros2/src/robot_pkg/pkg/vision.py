#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import json
import sys
from ultralytics import YOLO
from pathlib import Path

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ---------------------------------------------------
        # 1. PARAMETERS (The "Calibration" variables you asked for)
        # ---------------------------------------------------
        # Adjust these in your launch file or via command line later!
        # Example: ros2 run pkg vision_node --ros-args -p offset_x:=15.0
        self.declare_parameter('camera_offset_x', 0.0)
        self.declare_parameter('camera_offset_y', 0.0)
        self.declare_parameter('pixels_per_cm', 12.0)
        
        # ---------------------------------------------------
        # 2. LOAD MODEL
        # ---------------------------------------------------
        base_dir = Path(__file__).resolve().parent
        # Assuming you put weights in a 'weights' folder inside your pkg
        model_path = base_dir / "init/best.pt"  
        
        if not model_path.exists():
            self.get_logger().error(f"❌ Model not found at {model_path}")
            # Fallback for testing if custom path fails
            model_path = Path("best.pt") 

        self.get_logger().info(f"✅ Loading YOLO model: {model_path}")
        try:
            self.model = YOLO(str(model_path))
        except Exception as e:
            self.get_logger().fatal(f"Failed to load model: {e}")
            sys.exit(1)

        # ---------------------------------------------------
        # 3. SETUP CAMERA
        # ---------------------------------------------------
        self.cap = cv2.VideoCapture(5, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().warn("V4L2 failed, trying default...")
            self.cap = cv2.VideoCapture(5)
            
        if not self.cap.isOpened():
            self.get_logger().fatal("❌ No Camera Found!")
            sys.exit(1)

        # ---------------------------------------------------
        # 4. PUBLISHER
        # ---------------------------------------------------
        # Publishing a JSON string for easy parsing in the planner
        self.publisher_ = self.create_publisher(String, '/vision/detections', 10)
        
        # Create a timer to run the loop at 10Hz (0.1s)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("✅ Vision Node Started. Ready to detect.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Frame drop!")
            return

        # Get Parameters dynamically (so you can tune them while running)
        offset_x = self.get_parameter('camera_offset_x').value
        offset_y = self.get_parameter('camera_offset_y').value
        px_per_cm = self.get_parameter('pixels_per_cm').value
        
        # --- VISION LOGIC ---
        margin = 20
        results = self.model(frame, conf=0.7, verbose=False)
        
        detection_list = []

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                name = self.model.names[cls_id]
                
                # Center in Pixels (Image Coordinates)
                cx_pixel = (x1 + x2) // 2
                cy_pixel = (y1 + y2) // 2

                # ---------------------------------------------
                # COORDINATE TRANSFORMATION
                # ---------------------------------------------
                # 1. Convert to CM relative to Top-Left Margin
                cam_x_cm = (cx_pixel - margin) / px_per_cm
                cam_y_cm = (cy_pixel - margin) / px_per_cm
                
                # 2. Apply Calibration Offset to get Robot Frame
                robot_x = cam_x_cm + offset_x
                robot_y = cam_y_cm + offset_y
                
                # Add to list
                obj_data = {
                    "name": name,
                    "x": round(robot_x, 2),
                    "y": round(robot_y, 2),
                    "z": 0.0 # Assuming flat table for now
                }
                detection_list.append(obj_data)

                # Debug Drawing (Optional - remove if headless)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{name} ({robot_x:.1f}, {robot_y:.1f})"
                cv2.putText(frame, label, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # ---------------------------------------------------
        # PUBLISH DATA
        # ---------------------------------------------------
        if detection_list:
            msg = String()
            msg.data = json.dumps(detection_list) # Convert list to JSON string
            self.publisher_.publish(msg)
            # self.get_logger().info(f"Published: {msg.data}")

        # Show image (Only works if you have a display connected/forwarded)
        cv2.imshow("Robot Vision", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


    