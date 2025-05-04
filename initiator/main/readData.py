import serial
import csv
import re
from datetime import datetime

SERIAL_PORT = 'COM7'
BAUD_RATE = 115200
CSV_FILE = 'ftm_data.csv'

pattern = re.compile(
    r'FTM session complete\. Estimated RTT - (\d+) nSec, Estimated Distance - (\d+)\.(\d{2}) meters'
)

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, \
            open(CSV_FILE, 'w', newline='') as csvfile:

        writer = csv.writer(csvfile)
        writer.writerow(['Timestamp', 'RTT (nSec)', 'Distance (meters)'])

        print("Listening...")

        while True:
            try:
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                print(line)

                match = pattern.search(line)
                if match:
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    rtt = int(match.group(1))
                    distance = float(f"{match.group(2)}.{match.group(3)}")

                    writer.writerow([timestamp, rtt, distance])
                    csvfile.flush()
                    print(f"Write: {timestamp}, RTT: {rtt}, Distance: {distance:.2f} m")

            except KeyboardInterrupt:
                print("Exit with Keyboard Interrupt")
                break

if __name__ == '__main__':
    main()
