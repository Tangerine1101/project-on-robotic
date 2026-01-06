#!/usr/bin/env python3
#a small script to generate command packets in order to test serial communication, copy and run the output command to send data to the robot via serial

import struct
import json

# Packet configuration
CMD_ID = 'H'
BITMASK = 0x1F
TARGETS = [0.0, 0.0, 0.0, 30.0, 90.0]

# Protocol constants
START_BYTE = 0x23
PKG_FORMAT = '<B c B 5f'

def generate_packet():
    # Ensure exactly 5 targets
    safe_targets = (TARGETS + [0.0] * 5)[:5]

    try:
        payload = struct.pack(
            PKG_FORMAT,
            START_BYTE,
            CMD_ID.encode('ascii'),
            BITMASK,
            *safe_targets
        )
    except struct.error as e:
        print(f"Struct Error: {e}")
        return

    checksum = 0
    for byte in payload:
        checksum ^= byte

    final_packet = payload + struct.pack('<B', checksum)
    hex_output = final_packet.hex().upper()

    print(f"Cmd ID   : '{CMD_ID}'")
    print(f"Bitmask  : {bin(BITMASK)} ({BITMASK})")
    print(f"Args     : {safe_targets}")
    print(f"Checksum : {hex(checksum)}")

    print("HEX STRING (Raw):")
    print(hex_output)

    print(f'echo "{hex_output}" | xxd -r -p > /dev/ttyACM1')

def generate_ros2_vision_command():
    # Giả lập dữ liệu vật thể (onion hoặc garlic)
    # Tọa độ x, y tính bằng cm (vì vision.py gửi cm, planner nhân 10 để ra mm)
    fake_detections = [
        {
            "name": "onion",
            "x": 20.5,
            "y": 10.2,
            "z": 0.0
        }
    ]
    
    # Chuyển đổi sang chuỗi JSON
    json_string = json.dumps(fake_detections)
    
    # Escape dấu ngoặc kép để chạy được trong terminal
    escaped_json = json_string.replace('"', '\\"')
    
    print(f'ros2 topic pub -1 /vision/detections std_msgs/msg/String "{{data: \'{json_string}\'}}"')


if __name__ == "__main__":
    generate_packet()
    generate_ros2_vision_command()
    
