import random
import time

import serial

# Set the parameters for the serial connection
serial_port = '/dev/pts/12'  # Modify this to your serial port (e.g., 'COM3' on Windows)
baud_rate = 115200  # Modify this to your baud rate

# Open the serial connection
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