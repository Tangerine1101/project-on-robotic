import cv2
import sys
from ultralytics import YOLO
import random
from pathlib import Path

# -------------------------
# 1. DYNAMIC PATH SETUP
# -------------------------
# Finds the directory where THIS script is running
BASE_DIR = Path(__file__).resolve().parent

# Assumes best.pt is in the same folder or a 'weights' subfolder
# CHANGE THIS if your file is named differently
model_path = BASE_DIR / "init/best.pt" 

if not model_path.exists():
    print(f"❌ Error: Could not find model at {model_path}")
    print(f"   Current folder is: {BASE_DIR}")
    sys.exit(1)

print(f"✅ Loading model: {model_path}")
model = YOLO(str(model_path))

# -------------------------
# 2. LINUX CAMERA SETUP
# -------------------------
def open_camera():
    # Try V4L2 first (Standard for Linux)
    print("📷 Attempting to open camera with V4L2...")
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print("⚠️ V4L2 failed. Trying default backend...")
        cap = cv2.VideoCapture(0)
    
    if cap.isOpened():
        print("✅ Camera opened successfully.")
        return cap
    else:
        return None

# -------------------------
# CONSTANTS
# -------------------------
PIXELS_PER_CM = 12.0
MARGIN = 20
colors = {name: [random.randint(0, 255) for _ in range(3)] for name in model.names.values()}

# -------------------------
# MAIN LOOP
# -------------------------
if __name__ == "__main__":
    cap = open_camera()
    if cap is None:
        print("❌ Critical Failure: No webcam found.")
        print("   Did you pass '--device=/dev/video0:/dev/video0' to Docker?")
        sys.exit(1)

    print("✅ Webcam active. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Frame drop!")
            break

        annotated_frame = frame.copy()
        h, w = frame.shape[:2]

        # Draw Reference Box
        cv2.rectangle(annotated_frame, (MARGIN, MARGIN), (w - MARGIN, h - MARGIN), (0, 255, 0), 2)
        
        # Run YOLO
        results = model(frame, conf=0.7, verbose=False)

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                name = model.names[int(box.cls[0])]
                
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # Calculate CM
                cx_cm = (cx - MARGIN) / PIXELS_PER_CM
                cy_cm = (cy - MARGIN) / PIXELS_PER_CM
                
                # Draw
                color = colors[name]
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated_frame, f"{name} ({cx_cm:.1f}, {cy_cm:.1f}cm)", 
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow("YOLO Linux Test", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()