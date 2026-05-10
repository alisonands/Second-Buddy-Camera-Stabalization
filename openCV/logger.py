import serial
import csv
from datetime import datetime

PORT = "COM6"
BAUD = 115200

filename = datetime.now().strftime("sensor_log_%Y%m%d_%H%M%S.csv")

ser = serial.Serial(PORT, BAUD, timeout=1)

with open(filename, "w", newline="") as f:
    writer = csv.writer(f)

    # CSV header
    writer.writerow([
        "time",
        "desired_pitch",
        "measured_pitch",
        "desired_yaw",
        "measured_yaw"
    ])

    print(f"Logging to {filename}")

    while True:
        try:
            line = ser.readline().decode("utf-8").strip()

            if line:
                values = line.split(",")

                if len(values) == 5:
                    writer.writerow(values)
                    print(values)

        except KeyboardInterrupt:
            print("Stopped logging")
            break