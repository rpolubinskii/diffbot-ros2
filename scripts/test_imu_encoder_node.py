from time import sleep

import serial

ser = serial.Serial('/dev/imu-encoder-node', 115200, timeout=1)

while True:
    line = ser.readline().decode().strip()

    print(f"{line}")
