# Hướng dẫn vận hành – Cánh tay robot phân loại

Tài liệu này hướng dẫn **cài đặt, kết nối phần cứng và vận hành** hệ thống cánh
tay robot pick‑and‑place. Người đọc không cần biết lập trình; chỉ cần làm theo
từng bước. Muốn hiểu bên trong hoạt động ra sao, xem thêm
[`bao-cao-lap-trinh.md`](bao-cao-lap-trinh.md).

---

## 1. Hệ thống gồm những gì

Hệ thống có **2 nửa**, nối với nhau qua **1 dây USB**:

| Nửa | Chạy trên | Vai trò |
|-----|-----------|---------|
| **Firmware** (`src/`, `include/`) | Vi điều khiển **Arduino Due** | Điều khiển 3 động cơ bước (khớp 1‑2‑3) + 2 servo (khớp 4, kẹp). Nhận lệnh nhị phân qua USB. |
| **Ứng dụng PC** (`robot/`) | Máy tính (Windows hoặc Linux/WSL) | Camera + nhận diện vật (YOLO), tính toán quỹ đạo, gửi lệnh cho Due, và **giao diện web** để giám sát/điều khiển. |

**Luồng cơ bản:** camera thấy vật → PC nhận diện & tính vị trí (mm) → bộ lập lịch
quyết định gắp/nhả → gửi lệnh góc khớp cho Due → Due quay động cơ.

### Các công cụ (tool) đi kèm

| Lệnh | Công dụng |
|------|-----------|
| `python robot/main.py` | **Chạy đầy đủ**: camera + nhận diện + lập lịch + serial + web dashboard. |
| `python robot/main.py --no-vision` | Chạy không camera (điều khiển tay). |
| `python robot/main.py --manual` | Khởi động ở chế độ tay (không tự động phân loại). |
| `python robot/main.py --no-dashboard` | Không mở web dashboard. |
| `python robot/main.py -v` | Bật log chi tiết (DEBUG) để gỡ lỗi. |
| `python robot/tools/cli.py` | **Điều khiển dòng lệnh** trực tiếp (moveto/goto/grip/calibrate/monitor…). |
| `python robot/tools/test_camera.py` | Xem camera + nhận diện + ROI **không cần Due** (kiểm tra hiệu chuẩn). |
| `python robot/camera_calibrate.py` | **Hiệu chuẩn camera→robot** bằng bàn cờ (homography + vùng ROI). |
| `python robot/devices.py` | In ra cổng MCU và chỉ số camera mà hệ thống tự dò được. |

**Web dashboard** (mặc định `http://localhost:8000`) là giao diện vận hành chính:
xem camera trực tiếp, danh sách vật đang thấy, vị trí hiện tại của cánh tay, chuyển
chế độ Auto/Manual, và ở chế độ tay có nút Move / Grip / Release / Calibrate.

---

## 2. Yêu cầu

**Phần cứng:**
- Cánh tay robot đã lắp, cấp nguồn động cơ đầy đủ.
- Board **Arduino Due** đã nạp firmware (xem mục 6), nối PC bằng **cổng Programming** (USB gần jack nguồn).
- **Camera USB** (mã USB `0C45:636B`), gắn cố định nhìn xuống mặt bàn làm việc.

**Phần mềm (máy PC):**
- Python **3.10+**.
- Nếu dùng Windows và muốn môi trường Linux: cài **WSL2** (mục 3).
- (Để nạp firmware) **PlatformIO** hoặc Arduino IDE.

**Mã USB nhận dạng thiết bị** (hệ thống tự dò theo mã này):
- Arduino Due: `2341:003D`
- Camera: `0C45:636B`

Nếu tự dò sai, có thể chỉ định tay trong `robot/config.yaml` (`serial.port` và
`vision.camera_index`).

---

## 3. Cài đặt WSL trên Windows

> WSL2 cho phép chạy môi trường Ubuntu ngay trong Windows. **Lưu ý quan trọng:**
> Ứng dụng PC vốn chạy được **thẳng trên Windows** (không cần WSL). WSL tiện cho
> động cơ (cổng serial của Due) nhưng **camera USB thường KHÔNG hoạt động trong
> WSL2** nếu chưa biên dịch lại nhân Linux (xem mục 4). Vì vậy:
> - Muốn dùng camera dễ dàng → nên chạy **thẳng trên Windows** (mục 5B).
> - Chỉ cần điều khiển tay/không camera, hoặc chấp nhận cấu hình nhân → dùng WSL.

**Bước 3.1 – Cài WSL2 + Ubuntu.** Mở **PowerShell (Run as Administrator)**:
```powershell
wsl --install
```
Khởi động lại máy khi được yêu cầu. Lần đầu mở Ubuntu, đặt username/mật khẩu Linux.

**Bước 3.2 – Cập nhật & cài Python trong Ubuntu (WSL):**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3 python3-venv python3-pip git
```

**Bước 3.3 – Lấy mã nguồn và cài thư viện:**
```bash
git clone <đường-dẫn-repo> project-on-robotic
cd project-on-robotic/robot
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

---

## 4. Kết nối (bind) MCU và camera vào WSL

WSL2 không tự thấy thiết bị USB; phải "chuyển" (attach) từ Windows sang bằng công
cụ **usbipd-win**.

**Bước 4.1 – Cài usbipd-win (trên Windows).** Mở PowerShell (Admin):
```powershell
winget install usbipd
```
(Hoặc tải bản cài từ dự án `dorssel/usbipd-win` trên GitHub.)

**Bước 4.2 – Liệt kê thiết bị USB.** Cắm Due và camera, rồi chạy (PowerShell):
```powershell
usbipd list
```
Tìm 2 dòng: **Arduino Due** (`2341:003D`) và **camera** (`0C45:636B`). Ghi lại
`BUSID` của mỗi thiết bị (ví dụ `2-3`).

**Bước 4.3 – Chia sẻ & gắn vào WSL.** Với mỗi BUSID (PowerShell Admin):
```powershell
usbipd bind   --busid <BUSID>          # chỉ cần làm 1 lần cho mỗi thiết bị
usbipd attach --wsl --busid <BUSID>     # phải làm lại sau mỗi lần rút/khởi động lại
```
> `attach` cần WSL đang chạy (mở sẵn cửa sổ Ubuntu). Sau khi rút và cắm lại, hoặc
> khởi động lại máy, phải `attach` lại.

**Bước 4.4 – Kiểm tra trong WSL (Ubuntu):**
```bash
ls /dev/ttyACM*      # Due -> thường là /dev/ttyACM0
ls /dev/video*       # camera -> /dev/video0 (nếu nhân hỗ trợ)
```
Cấp quyền truy cập cho user (một lần, rồi mở lại terminal):
```bash
sudo usermod -aG dialout,video $USER
```

**Bước 4.5 – Về camera trong WSL.** Nhân WSL2 mặc định thường **thiếu driver
UVC (USB Video Class)** nên `/dev/video0` không xuất hiện. Hai lựa chọn:
- **(Khuyến nghị) Chạy phần camera thẳng trên Windows** (mục 5B) — code đã hỗ trợ
  sẵn (dùng DirectShow trên Windows).
- Hoặc biên dịch lại nhân WSL có bật `CONFIG_USB_VIDEO_CLASS` (nâng cao, ngoài
  phạm vi tài liệu này).

Xác nhận nhanh thiết bị được nhận:
```bash
python devices.py    # in ra "MCU port" và "Camera index"
```

---

## 5. Chạy chương trình

Trước khi chạy: đảm bảo động cơ đã cấp nguồn, không có người/vật cản trong vùng
hoạt động của cánh tay.

### 5A. Trên WSL / Linux
```bash
cd project-on-robotic/robot
source .venv/bin/activate
python main.py                 # đầy đủ (nếu camera đã nhận trong WSL)
# hoặc, nếu chưa có camera trong WSL:
python main.py --no-vision     # chỉ điều khiển tay
```

### 5B. Thẳng trên Windows (khuyến nghị khi dùng camera)
Mở **PowerShell/CMD** (không cần WSL, không cần usbipd):
```powershell
cd project-on-robotic\robot
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python main.py
```
Windows tự thấy cổng COM của Due và camera; code dùng DirectShow cho camera.

### Mở dashboard
Sau khi thấy log `dashboard on http://0.0.0.0:8000`, mở trình duyệt:
```
http://localhost:8000
```

---

## 6. Nạp firmware cho Arduino Due (khi cần)

Chỉ làm khi thay đổi mã firmware hoặc board mới. Dùng **PlatformIO** (khuyến nghị):
```bash
pio run              # biên dịch
pio run -t upload    # nạp qua cổng Programming của Due
```
Hoặc Arduino IDE: chép toàn bộ file trong `src/` và `include/` vào một thư mục tên
`main`, đổi `main.cpp` → `main.ino`, mở bằng Arduino IDE, cài thư viện
**AccelStepper** và **Servo**, chọn board Arduino Due (Programming Port), Upload.

> **Quan trọng:** Firmware và ứng dụng PC phải khớp phiên bản gói tin (gói trạng
> thái hiện dài 25 byte, có byte công tắc hành trình). Nếu cập nhật một bên, nạp
> lại cả hai.

---

## 7. Hiệu chuẩn camera → robot (bắt buộc trước khi chạy Auto)

Bộ nhận diện trả về vị trí vật theo **pixel**; hệ thống cần biết đổi pixel sang
**tọa độ robot (mm)**. Việc này làm 1 lần bằng **bàn cờ (checkerboard)** và lưu vào
`robot/camera_calibration.yaml`. Quy trình **2 lượt, KHÔNG được xê dịch bàn cờ giữa
2 lượt**:

**Chuẩn bị:** in bàn cờ (`tools/make_checkerboard.py`), đo lại cạnh ô thật và cập
nhật `square_mm` trong `robot/checkerboard.json` (mặc định 20 mm, lưới 7×11 góc
trong). Đặt bàn cờ nằm phẳng trong vùng làm việc.

**Lượt 1 – dò bàn cờ:**
```bash
python camera_calibrate.py
```
Chương trình lưu ảnh `dataset/calib/checkerboard_detected.jpg` có đánh dấu **3 góc
BL / BR / TL** (hình chữ L). Dùng đầu robot chạm vào **đúng 3 góc vật lý đó**, đọc
tọa độ **X, Y (mm)** của robot tại mỗi góc (dùng `tools/cli.py` để rê tay), rồi điền
vào `robot/checkerboard.json` mục `robot_points_mm`.

**Lượt 2 – tính & ghi hiệu chuẩn:** giữ nguyên bàn cờ, chạy lại:
```bash
python camera_calibrate.py
```
Nó khớp ma trận **homography** (pixel→mm) + **vùng ROI** (khung làm việc) và ghi
`camera_calibration.yaml`, kèm ảnh kiểm chứng `dataset/calib/checkerboard_calibrated.jpg`.

**Kiểm tra:**
```bash
python tools/test_camera.py
```
Đặt vài vật: nhãn **(x, y) mm** (màu cam) phải khớp thực tế khi đối chiếu bằng đầu
robot; đường viền ROI (vàng) đúng vùng bàn; vật ngoài ROI bị đánh dấu khác màu.

> Đổi độ phân giải camera **không cần** hiệu chuẩn lại: hệ thống tự quy đổi
> (`vision.py` xử lý cắt/thu phóng theo tỉ lệ khung hình).

---

## 8. Dùng web dashboard

Giao diện một màn hình, tiếng Anh, gồm:

- **Thanh trên cùng – công tắc chế độ lớn:**
  - **AUTO** – chạy chu trình phân loại tự động.
  - **MANUAL** – bạn tự điều khiển; đây cũng là cách **dừng** chu trình auto.
  - Các "pill" hiển thị: chế độ hiện tại, việc đang làm (bằng chữ dễ hiểu), tình
    trạng kết nối.
- **Bên trái – Camera:** ảnh trực tiếp có khung nhận diện, và danh sách **Detected
  objects** (tên vật + tọa độ `(x, y) mm`).
- **Bên phải – Current position:** góc 4 khớp + góc kẹp, trạng thái 3 công tắc hành
  trình A/B/C, tình trạng kết nối.
- **Bên phải – Manual control** (chỉ dùng được khi ở MANUAL; ở AUTO sẽ mờ đi):
  1. **Go to point** – nhập X, Y, Z (mm), bấm **Move**: hệ thống tự tính góc khớp
     (IK) và di chuyển. Điểm ngoài tầm với sẽ báo *unreachable*.
  2. **Gripper** – nút **Grip** (kẹp) / **Release** (nhả).
  3. **Calibrate** – chạy hiệu chuẩn về công tắc gốc (~vài chục giây).
  4. **Advanced: joint angles** – nhập trực tiếp góc J1–J4 (dành cho người rành).
- **Diagnostics** (gập lại): biểu đồ góc khớp 1 phút gần nhất.

**Chế độ AUTO** tự động: vào auto → **hiệu chuẩn một lần** → lặp: tìm vật gần hộp
nhất → đi tới **phía trên vật** → hạ xuống gắp → nhấc lên → sang **trên hộp** → hạ
xuống nhả → nhấc lên → sang vật kế tiếp. Hết vật thì về **home** và đứng chờ.

---

## 9. Điều khiển bằng dòng lệnh (`tools/cli.py`)

Khi cần thao tác thủ công/chẩn đoán mà không qua web:
```bash
python tools/cli.py
```
Các lệnh chính:

| Lệnh | Ý nghĩa |
|------|---------|
| `goto <x> <y> <z>` | Di chuyển tới điểm (mm) qua IK (khớp 1‑4). |
| `moveto -a <độ> -b <độ> -c … -d … -e …` | Di chuyển tuyệt đối theo góc từng khớp. |
| `move -a <độ> …` | Di chuyển tương đối. |
| `grip` / `release` | Đóng / mở kẹp. |
| `calibrate` | Hiệu chuẩn về gốc. |
| `position` | In góc hiện tại + trạng thái công tắc. |
| `monitor` | In luồng trạng thái 20 Hz (Ctrl‑C để dừng). |
| `help` / `quit` | Trợ giúp / thoát. |

---

## 10. Cấu hình (`robot/config.yaml`)

Các mục hay chỉnh:

- **`serial`**: `port` (`auto` hoặc `COM5`/`/dev/ttyACM0`), các timeout.
- **`vision`**: `camera_index` (`auto` hoặc số), `frame_width/height` (đang 640×480),
  `model_path` (`best.pt`), `conf_threshold` (ngưỡng tin cậy nhận diện).
- **`dashboard`**: `host`/`port` (mặc định `0.0.0.0:8000`).
- **`planner`** (bộ lập lịch):
  - `lift_height_mm` – độ cao an toàn khi nhấc/di chuyển ngang (mm).
  - `drop_height_mm` – **độ cao hạ xuống hộp; PHẢI đo theo miệng hộp thật.**
  - `zones` – tọa độ hộp phân loại cho từng loại (`onion`/`garlic`/`lemon`).
  - `zone_radius_mm` – bán kính coi như "vật đã nằm trong hộp" (bỏ qua, không gắp).
  - `home_angles` – tư thế nghỉ.

---

## 11. Xử lý sự cố

| Hiện tượng | Cách xử lý |
|-----------|-----------|
| `MCU ... not found` | Kiểm tra dây USB (cổng Programming), quyền `dialout`; trong WSL nhớ `usbipd attach`; hoặc đặt `serial.port` tay. |
| `Camera ... not found` / không có ảnh | Trong WSL camera thường không có `/dev/video0` → chạy thẳng trên Windows; hoặc đặt `vision.camera_index` tay. |
| `calibration file ... not found` | Chưa hiệu chuẩn camera → làm mục 7. |
| Nhận diện vị trí lệch | Chạy lại hiệu chuẩn (mục 7); kiểm tra bằng `test_camera.py`. |
| Cánh tay không tới điểm / báo *unreachable* | Điểm ngoài vùng an toàn của IK — chọn điểm khác hoặc chỉnh `zones`/độ cao. |
| Auto không gắp vật | Vật nằm ngoài ROI, hoặc nằm sẵn trong bán kính hộp (`zone_radius_mm`), hoặc độ tin cậy < `conf_threshold`. |
| Cần xem chi tiết đang xảy ra gì | Chạy `python main.py -v` và đọc log (dòng `state:`, `[hb] task=… pos=…`, `move: waiting residual[…]`). |

---

## 12. An toàn

- Luôn có **nút ngắt nguồn động cơ** trong tầm tay khi chạy thử.
- Chuyển **MANUAL** trên dashboard để dừng chu trình auto (lệnh `abort` của firmware
  **chưa hoàn thiện**, không đảm bảo dừng cơ khí — xem `doc/bug-report.md` 5.1).
- Lần đầu chạy, đặt tốc độ chậm và đứng gần công tắc nguồn; kiểm tra vùng làm việc
  trống trước khi vào AUTO.
- Đo và đặt đúng `drop_height_mm` theo miệng hộp thật để tránh đâm kẹp xuống hộp.
