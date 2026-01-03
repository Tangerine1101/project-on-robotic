import serial
import struct
import time

# --- CONFIGURATION (Updated from your config.h) ---
# Checked against config.h: #define maxArguments 5
MAX_ARGUMENTS = 5 
# Checked against config.h: #define NODE_SENDBYTE '@'
# ASCII '@' is 0x40. This is what starts the packet FROM the MCU.
NODE_SENDBYTE = b'@' 

PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- COMMAND MAPPING (Updated from serialCommand.h) ---
# These match the 'commands' enum in your header.
COMMAND_MAP = {
    '~': "None/Idle",
    'M': "Move (Relative)",      # cmd_move
    'A': "MoveTo (Absolute)",    # cmd_moveto
    'P': "Position Report",      # cmd_position
    'C': "Set Current Pos",      # cmd_currentPos
    'G': "Grip (Close)",         # cmd_grip
    'R': "Release (Open)",       # cmd_release
    'F': "MoveRef (Calibrate)",  # cmd_moveref
    'H': "Human Interface",      # cmd_humanInterface
    'S': "ROS2 Interface",       # cmd_ros2Interface
    'X': "Abort",                # cmd_abort
    '&': "Invalid"               # cmd_invalid
}

# --- STATUS MAPPING ---
# Based on your operate() function logic
STATUS_MAP = {
    'P': "Processing",
    'D': "Done",
    'F': "Fail",
    '~': "Idle"
}

# --- STRUCT FORMAT ---
# < : Little Endian
# B : uint8_t (startByte)
# c : char (processingID) - receiving as char/byte
# c : char (statusID)     - receiving as char/byte
# 5f: float Arguments[5]
# B : uint8_t (checksum)
STRUCT_FMT = f'<Bcc{MAX_ARGUMENTS}fB'
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)

def calculate_checksum_xor(data_bytes):
    """
    Matches C++: checksumXOR(uint8_t* data, size_t length)
    """
    checksum = 0
    # XOR all bytes EXCEPT the last one (which is the received checksum)
    for b in data_bytes[:-1]:
        checksum ^= b
    return checksum

def main():
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {PORT}. Waiting for start byte '{NODE_SENDBYTE.decode()}'...")
        
        ser.reset_input_buffer()

        while True:
            # 1. HEADER SEARCH (Sync)
            if ser.in_waiting > 0:
                byte = ser.read(1)
                
                # Check if this byte is the Start Byte ('@')
                if byte == NODE_SENDBYTE:
                    # We found the start! Now read the rest of the struct.
                    remaining_size = STRUCT_SIZE - 1
                    
                    # Block briefly until we have the full packet
                    # (In a rigorous UI, you'd use a state machine, but this works for testing)
                    while ser.in_waiting < remaining_size:
                        pass 
                    
                    rest_of_data = ser.read(remaining_size)
                    full_packet = byte + rest_of_data
                    
                    try:
                        # 2. UNPACK
                        unpacked = struct.unpack(STRUCT_FMT, full_packet)
                        
                        # Extract Raw Fields
                        # unpacked[1] and [2] come out as bytes (e.g., b'M') because of 'c' format
                        proc_id_byte = unpacked[1] 
                        status_id_byte = unpacked[2]
                        arguments = unpacked[3:-1]
                        recv_checksum = unpacked[-1]

                        # 3. VERIFY CHECKSUM
                        calc_sum = calculate_checksum_xor(full_packet)
                        
                        if calc_sum == recv_checksum:
                            # Decode bytes to string for lookup
                            proc_char = proc_id_byte.decode('utf-8', errors='ignore')
                            status_char = status_id_byte.decode('utf-8', errors='ignore')

                            # Get readable text
                            proc_str = COMMAND_MAP.get(proc_char, f"Unknown ({proc_char})")
                            status_str = STATUS_MAP.get(status_char, f"Unknown ({status_char})")

                            # 4. PRINT OUTPUT
                            print("-" * 40)
                            print(f"Command      : {proc_str}")
                            print(f"Status       : {status_str}")
                            # Format floats nicely (2 decimal places)
                            args_formatted = ", ".join([f"{x:6.2f}" for x in arguments])
                            print(f"Arguments    : [{args_formatted}]")
                            print(f"Checksum     : OK ({recv_checksum:02X})")
                        
                        else:
                            print(f"[Error] Checksum Mismatch! Calc: {calc_sum:02X} != Recv: {recv_checksum:02X}")
                            print(f"Raw Packet: {full_packet.hex()}")

                    except struct.error as e:
                        print(f"[Critical] Unpack error: {e}")

    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
        ser.close()

if __name__ == "__main__":
    main()