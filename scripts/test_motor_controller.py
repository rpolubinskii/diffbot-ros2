import serial

# Adjust the serial port name for your Arduino
ser = serial.Serial('/dev/motor-controller', 115200, timeout=1)

print("Enter commands like: 200,200, -150,140")
try:
    while True:
        cmd = input("> ").strip()
        if cmd:
            print(f"Writing {cmd}")
            ser.write((cmd + "\n").encode())
except KeyboardInterrupt:
    print("\nExiting...")