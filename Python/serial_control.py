import serial
import time


# Set up the serial port and baud rate; ensure it matches the Arduino settings

def connect_serial(com='COM12', rate=11520):
    ser = serial.Serial(com, rate)  # Replace 'COM3' with your actual serial port
    # Commands and corresponding delay times (in seconds)
    return ser


def write_w_delay(commands_with_delays):
    a = 0
    for command, delay in commands_with_delays:
        a += 1;
        ser.write(f"{command}\n".encode())  # Send command
        print(f"Sent command: {command}")
        print(f"a: {a}")
        time.sleep(delay)  # Wait for the specified delay time


if __name__ == '__main__':

    commands_with_delays = [
        ('c0', 0.5),  # Send 'b0.8' and wait 5 seconds
        ('c10', 0.1)
    ]

    ser = connect_serial('COM12', 115200)

    try:
        while True:
            write_w_delay(commands_with_delays)

    except KeyboardInterrupt:
        print("Program stopped")
        ser.write(f"{'a0'}\n".encode())
        ser.write(f"{'b0'}\n".encode())
        ser.write(f"{'c0'}\n".encode())
        ser.write(f"{'d0'}\n".encode())
    finally:
        ser.close()  # Close the serial connection
