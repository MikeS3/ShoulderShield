import serial
import time
from datetime import datetime


PORT = 'COM6'         # Replace with Pico COM port'COM5' or '/dev/ttyACM0'
BAUD = 115200
TIMEOUT = 1.0         # Seconds


def get_timestamped_filename():
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    return f"IMU_data_{timestamp}.sto"

def wait_for_prompt(ser, prompt="# Waiting for start"):
    """Waits until the Pico is ready to receive the start command."""
    print("[INFO] Waiting for device prompt...")
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if prompt in line:
            print("[INFO] Device ready. Sending start signal.")
            return
        elif line:
            print(f"[USB] {line}")

def main():
    print(f"[INFO] Connecting to {PORT} at {BAUD} baud...")
    try:
        with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
            #wait_for_prompt(ser)

            # Send start
            ser.write(b"start\n")

            filename = get_timestamped_filename()
            print(f"[INFO] Logging data to: {filename}")

            with open(filename, 'w', encoding='utf-8') as f:
                while True:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    # Save line to file
                    f.write(line + '\n')
                    f.flush()
                    print(line)  # Optional: show on screen

                    # Stop if the device prints a pause message
                    if line.startswith("# Data collection paused"):
                        print("[INFO] Collection stopped by device.")
                        break

    except serial.SerialException as e:
        print(f"[ERROR] Could not open serial port {PORT}: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user. Exiting.")

if __name__ == "__main__":
    main()
