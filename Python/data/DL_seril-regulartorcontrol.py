import serial
import time

if __name__ == '__main__':
    with serial.Serial(port='COM8', baudrate=115200, timeout=1) as ser:


        commands = ["d2", "c2", "b2", "a2"]
        command = "d2"
        for command in commands:
            command+='\r'
            ser.write((command).encode('utf-8'))
            print(f"Sent: {command.strip()}")
            time.sleep(1)  # Wait for 2 seconds

        commands = ["d2", "a" "c2", "d", "b2", "c", "a2", "b"]
        while True:
            for command in commands:
                ser.write(command.encode('utf-8'))  # Send command
                print(f"Sent: {command.strip()}")
                time.sleep(2)  # Wait for 2 seconds
            # Check for stop condition
            if input("Type 'stop' to end or press Enter to continue: ").strip().lower() == 'stop':
                print("Stopping the command sending.")
                break