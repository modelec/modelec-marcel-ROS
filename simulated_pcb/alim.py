import random
import time

import serial
import argparse



parser = argparse.ArgumentParser(description="Simulated PCB")
parser.add_argument('--port', type=str, default='/dev/pts/6', help='Serial port to use')
args = parser.parse_args()

serial_port = args.port
baud_rate = 115200

ser = serial.Serial(serial_port, baud_rate, timeout=1)

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()

            if data.startswith("BAU;STATE"):
                print("Received:", data)
                ser.write(b'SET;BAU;STATE;1\n')

        time.sleep(1)
except KeyboardInterrupt:
    print("Program interrupted by user.")
finally:
    # Close the serial connection
    ser.close()

# socat -d -d pty,raw,echo=0 pty,raw,echo=0
