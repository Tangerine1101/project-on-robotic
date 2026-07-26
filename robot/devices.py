"""Auto-detect the MCU and camera by USB vendor:product ID.

MCU:    2341:003D  (Arduino Due, programming port)
Camera: 0c45:636b

Both are overridable via config.yaml (serial.port / vision.camera_index) if
auto-detection ever misbehaves on a given machine.
"""
import glob
import os
import sys

import serial.tools.list_ports as list_ports

MCU_VID, MCU_PID = 0x2341, 0x003D
CAMERA_VID, CAMERA_PID = 0x0C45, 0x636B


class DeviceNotFoundError(RuntimeError):
    pass


def find_mcu_port():
    """Cross-platform: returns 'COM3' on Windows, '/dev/ttyACM0' on Linux, etc."""
    for p in list_ports.comports():
        if p.vid == MCU_VID and p.pid == MCU_PID:
            return p.device
    raise DeviceNotFoundError(
        f"MCU (USB {MCU_VID:04x}:{MCU_PID:04x}) not found. "
        "Is the Due plugged in? Or set serial.port explicitly in config.yaml."
    )


def find_camera_index():
    if sys.platform.startswith("linux"):
        return _find_camera_index_linux()
    if sys.platform == "win32":
        return _find_camera_index_windows()
    raise DeviceNotFoundError(
        "Camera auto-detect isn't implemented for this platform. "
        "Set vision.camera_index explicitly in config.yaml."
    )


def _read_id(path):
    try:
        with open(path) as f:
            return f.read().strip().lower()
    except OSError:
        return None


def _find_camera_index_linux():
    nodes = sorted(
        glob.glob("/sys/class/video4linux/video*"),
        key=lambda p: int(p.rsplit("video", 1)[1]),
    )
    for node in nodes:
        dev_path = os.path.realpath(os.path.join(node, "device"))
        vid = pid = None
        p = dev_path
        for _ in range(6):  # walk up the sysfs device chain to the USB device node
            v, d = _read_id(os.path.join(p, "idVendor")), _read_id(os.path.join(p, "idProduct"))
            if v and d:
                vid, pid = v, d
                break
            parent = os.path.dirname(p)
            if parent == p:
                break
            p = parent
        if vid == f"{CAMERA_VID:04x}" and pid == f"{CAMERA_PID:04x}":
            return int(node.rsplit("video", 1)[1])
    raise DeviceNotFoundError(
        f"Camera (USB {CAMERA_VID:04x}:{CAMERA_PID:04x}) not found. "
        "Set vision.camera_index explicitly in config.yaml."
    )


def _find_camera_index_windows():
    # Best-effort: match a USB VID/PID via WMI, then correlate its position
    # among capture devices with the index cv2.VideoCapture(index, CAP_DSHOW)
    # expects. NOT verified on real Windows hardware (this was written and
    # tested on a Linux dev machine) -- if it misbehaves, just set
    # vision.camera_index to an integer in config.yaml instead.
    try:
        import wmi
    except ImportError as e:
        raise DeviceNotFoundError(
            "Camera auto-detect on Windows requires the 'wmi' package "
            "(pip install wmi pywin32), or set vision.camera_index in config.yaml."
        ) from e

    c = wmi.WMI()
    target = f"vid_{CAMERA_VID:04x}&pid_{CAMERA_PID:04x}"
    all_cameras = list(c.Win32_PnPEntity(PNPClass="Camera"))
    matches = [d for d in all_cameras if d.DeviceID and target in d.DeviceID.lower()]
    if not matches:
        raise DeviceNotFoundError(
            f"Camera (USB {CAMERA_VID:04x}:{CAMERA_PID:04x}) not found via WMI. "
            "Set vision.camera_index explicitly in config.yaml."
        )
    for idx, d in enumerate(all_cameras):
        if d in matches:
            return idx
    return 0


if __name__ == "__main__":
    try:
        print("MCU port:", find_mcu_port())
    except DeviceNotFoundError as e:
        print("MCU:", e)
    try:
        print("Camera index:", find_camera_index())
    except DeviceNotFoundError as e:
        print("Camera:", e)
