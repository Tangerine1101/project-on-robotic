#4 
import cv2
import math

# ========= THÔNG SỐ THỰC =========
REAL_DISTANCE_CM = 12.0   # đổi thành kích thước vật chuẩn (cm)

points = []

def mouse_click(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        print(f"📍 Point {len(points)}: ({x}, {y})")

cap = cv2.VideoCapture(0)
cv2.namedWindow("Calibration")
cv2.setMouseCallback("Calibration", mouse_click)

print("👉 Click 2 điểm trên vật chuẩn (biết trước kích thước cm)")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Vẽ điểm
    for p in points:
        cv2.circle(frame, p, 5, (0, 0, 255), -1)

    # Khi đủ 2 điểm
    if len(points) == 2:
        x1, y1 = points[0]
        x2, y2 = points[1]

        pixel_dist = math.hypot(x2 - x1, y2 - y1)
        cm_per_pixel = REAL_DISTANCE_CM / pixel_dist
        pixel_per_cm = pixel_dist / REAL_DISTANCE_CM

        cv2.line(frame, points[0], points[1], (255, 0, 0), 2)

        text = f"{pixel_dist:.1f}px = {REAL_DISTANCE_CM}cm"
        cv2.putText(frame, text, (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        print("\n===== KẾT QUẢ HIỆU CHUẨN =====")
        print(f"1 cm = {pixel_per_cm:.2f} pixel")
        print(f"100 pixel = {100 * cm_per_pixel:.2f} cm")
        print("=============================\n")

    cv2.imshow("Calibration", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
