import serial
import time
import re
from datetime import datetime
import os

# Change port to the serial port the Pico is connected to
def parse_serial_triplets(
    port='COM6',
    baudrate=115200,
    num_measurements=200,
    debug=False
):

    # Define the directory to save data
    data_dir = "data"

    # Create the directory if it doesn't exist
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)
        print(f"Created directory: '{data_dir}'")

    # Generate timestamped filename and join it with the directory path
    timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"imu_data_{timestamp_str}.sto"
    output_file = os.path.join(data_dir, filename)

    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Listening on {port}... Waiting for <data_start> with rate...")

    # Wait for "<data_start>" and extract rate
    while True:
        line = ser.readline().decode('utf-8').strip()
        if "<data_start>" in line and "rate:" in line:
            match = re.search(r"rate[: ]+([\d\.]+)", line)
            if match:
                rate = float(match.group(1))
                print(f"Data rate detected: {rate} Hz")
                break

    latest = {}
    has_new_data = {"scapula_imu": False, "sternum_imu": False, "humerus_imu": False}
    collected_rows = []

    while len(collected_rows) < num_measurements:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            if debug:
                print(f"> {line}")

            try:
                timestamp_str, label, w, x, y, z = line.split(",")
                label = label.strip()
                if label not in has_new_data:
                    continue

                timestamp = float(timestamp_str)
                quat = f"{float(w):.6f}, {float(x):.6f}, {float(y):.6f}, {float(z):.6f}"
                latest[label] = (timestamp, quat)
                has_new_data[label] = True

                if all(has_new_data.values()):
                    timestamps = [latest[imu][0] for imu in has_new_data]
                    row_time = round(sorted(timestamps)[1], 3) # median
                    row = {
                        "time": row_time,
                        "scapula_imu": latest["scapula_imu"][1],
                        "sternum_imu": latest["sternum_imu"][1],
                        "humerus_imu": latest["humerus_imu"][1],
                    }
                    collected_rows.append(row)

                    if debug:
                        print(f"Row {len(collected_rows)} @ {row_time:.3f}")

                    for imu in has_new_data:
                        has_new_data[imu] = False

            except ValueError:
                if debug:
                    print(f"Skipping invalid line: {line}")

        except KeyboardInterrupt:
            print("⏹ Interrupted by user.")
            break

    ser.close()

    # Save to file
    with open(output_file, "w") as f:
        f.write(f"DataRate={rate:.2f}\n")
        f.write("DataType=Quaternion\n")
        f.write("version=3\n")
        f.write("OpenSimVersion=4.5\n")
        f.write("endheader\n")
        f.write("time\tscapula_imu\tsternum_imu\thumerus_imu\n")

        for row in collected_rows:
            f.write(f"{row['time']:.3f}\t{row['scapula_imu']}\t{row['sternum_imu']}\t{row['humerus_imu']}\n")

    print(f"Saved {len(collected_rows)} synchronized rows to '{output_file}'.")

# Run the function
if __name__ == "__main__":
    parse_serial_triplets(debug=False)