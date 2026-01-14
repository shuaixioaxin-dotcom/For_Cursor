import serial
import time
import re

def parse_imu_line(line):
    """
    Parse a single line of IMU data.
    Expected format: [IMUx]:q0,q1,q2,q3
    Example: [IMU1]:-0.394,0.645,0.277,0.594
    """
    # Regex to match [IMU x]: float, float, float, float
    # Handling optional spaces and potential negative signs
    pattern = r"\[IMU\s*(\d+)\]:\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)"
    match = re.search(pattern, line)
    
    if match:
        imu_id = int(match.group(1))
        q0 = float(match.group(2))
        q1 = float(match.group(3))
        q2 = float(match.group(4))
        q3 = float(match.group(5))
        return imu_id, (q0, q1, q2, q3)
    return None

def main():
    serial_port = 'COM52'  # Port specified by user
    baud_rate = 115200     # Baud rate specified by user
    
    print(f"Connecting to {serial_port} at {baud_rate} baud...")
    
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print("Connected successfully. Press Ctrl+C to exit.")
        print("-" * 50)
        print(f"{'Time':<12} | {'IMU ID':<6} | {'Quaternion (w, x, y, z)':<30}")
        print("-" * 50)

        while True:
            if ser.in_waiting > 0:
                # Read a line and decode it
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                except Exception as e:
                    print(f"Error reading line: {e}")
                    continue
                
                if not line:
                    continue

                # Parse the line
                result = parse_imu_line(line)
                
                if result:
                    imu_id, quat = result
                    timestamp = time.strftime("%H:%M:%S", time.localtime())
                    # quat is (w, x, y, z) corresponding to q0, q1, q2, q3
                    quat_str = f"[{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]"
                    print(f"{timestamp:<12} | IMU {imu_id}  | {quat_str}")
                
                # Optionally handle FPS or other messages if needed
                elif "[FPS]" in line:
                    print(f"\033[92m{line}\033[0m") # Print FPS in green if terminal supports it

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("\nExiting program...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
