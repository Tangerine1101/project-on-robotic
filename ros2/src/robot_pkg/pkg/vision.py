#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import sys
from ultralytics import YOLO
from pathlib import Path

# Import from shared interface
from robot_interfaces.msg import ObjectList, DetectedObject 

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # --- PARAMETERS ---
        self.declare_parameter('x_0_cm', -7.0) 
        self.declare_parameter('y_0_cm', 5.5)   
        self.declare_parameter('pixels_per_cm', 10.9)
        self.declare_parameter('camera_dev', 4)        
        self.declare_parameter('conf_threshold', 0.7)
        
        # NEW PARAMETER: Stream Frequency
        self.declare_parameter('stream_frequency', 24) 

        # --- MODEL LOADING ---
        base_dir = Path(__file__).resolve().parent
        model_path = base_dir / "init/best.pt"  
        if not model_path.exists():
            model_path = Path("best.pt") 

        self.get_logger().info(f"Loading Model: {model_path}")
        try:
            self.model = YOLO(str(model_path))
        except Exception as e:
            self.get_logger().fatal(f"Model Load Failed: {e}")
            sys.exit(1)

        # --- CAMERA SETUP ---
        cam_id = self.get_parameter('camera_dev').value
        self.cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cam_id)
            
        if not self.cap.isOpened():
            self.get_logger().fatal(f"❌ Camera {cam_id} not found!")
            sys.exit(1)

        # --- PUBLISHER & TIMER ---
        self.publisher_ = self.create_publisher(ObjectList, '/vision/detections', 10)
        
        # Get frequency and convert to period (seconds)
        freq = self.get_parameter('stream_frequency').value
        if freq <= 0: freq = 10.0 # Safety default
        period = 1.0 / freq
        
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info(f"👁️ Vision Node Online on /dev/video{cam_id} @ {freq}Hz")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        # 1. Update Params
        x_0 = self.get_parameter('x_0_cm').value
        y_0 = self.get_parameter('y_0_cm').value
        px_per_cm = self.get_parameter('pixels_per_cm').value
        conf_thresh = self.get_parameter('conf_threshold').value

        # 2. Geometry
        h, w = frame.shape[:2]
        ref_x = w // 2
        ref_y = h - 30 
        robot_origin_pixel_x = int(ref_x - (y_0 * px_per_cm))
        robot_origin_pixel_y = int(ref_y - (x_0 * px_per_cm))

        # 3. Inference
        results = self.model(frame, conf=conf_thresh, verbose=False)
        
        msg = ObjectList()
        msg.objects = [] 

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                name = self.model.names[cls_id]
                
                cx_pixel = (x1 + x2) // 2
                cy_pixel = (y1 + y2) // 2

                # Transform
                obj_x_cm = (robot_origin_pixel_y - cy_pixel) / px_per_cm
                obj_y_cm = (robot_origin_pixel_x - cx_pixel) / px_per_cm

                obj = DetectedObject()
                obj.name = name
                obj.x = float(obj_x_cm)
                obj.y = float(obj_y_cm)
                obj.z = 0.0
                msg.objects.append(obj)

                # Debug Draw
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{name}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # 4. Publish
        self.publisher_.publish(msg)
        
        # Display
        cv2.circle(frame, (robot_origin_pixel_x, robot_origin_pixel_y), 8, (0, 0, 255), -1)
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
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()