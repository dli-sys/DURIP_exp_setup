import serial
import time
# Set up the serial port and baud rate; ensure it matches the Arduino settings
ser = serial.Serial('COM12', 115200)  # Replace 'COM3' with your actual serial port
# Commands and corresponding delay times (in seconds)

if __name__ == '__main__':

    a=0;
    commands_with_delays = [

    # DL command for bivalves
        ('c0', 0.5),  # Send 'b0.8' and wait 5 seconds
        # ('b10', 0.1),   # Send 'b0.8' and wait 5 seconds
        # ('c10', 5),   # Send 'c0.8' and wait 3 seconds
        # ('a2', 0.5),
        ('c10', 0.1),
        # ('c0', 0.14),
        # ('c0', 1),
        # ('b2.3', 1),  # Send 'c0' and wait 6 seconds
        # # ('c2', 1),
        # ('a0', 0.1),

        # ('c0', 0.1),  # Send 'b0.8' and wait 5 seconds
        # # ('b10', 0.1),   # Send 'b0.8' and wait 5 seconds
        # # ('c10', 5),   # Send 'c0.8' and wait 3 seconds
        # ('a2', 0.5),
        # ('c2', 0.1),
        # ('a0', 0.5),
        # ('c0', 1),
        # ('b2.3', 1),  # Send 'c0' and wait 6 seconds
        # # ('c2', 1),
        # ('a0', 0.1),
        # ('b0', 3),


        # ('a0', 0.1),   # Send 'b0.8' and wait 5 seconds
        # # ('b10', 0.1),   # Send 'b0.8' and wait 5 seconds
        # # ('c10', 5),   # Send 'c0.8' and wait 3 seconds
        # ('a2', 1),
        # # ('c0', 1),
        # ('b2.3', 1),# Send 'c0' and wait 6 seconds
        # # ('c2', 1),
        # ('a0', 0.1),
        # ('b0', 3),


        # ('b10', 1),
        # ('b10', 1)
        # Send 'b0' and wait 4 seconds
        # ('b10', 2),   # Send 'b0.8' and wait 5 seconds
        # ('c0', 0.5),      # Send 'c0' and wait 6 seconds
        # ('c3', 15),   # Send 'c0.8' and wait 3 seconds
        # ('b3.5', 30),   # Send 'b0.8' and wait 5 seconds
        # ('c0', 15),      # Send 'c0' and wait 6 seconds
        # ('b0', 60),     # Send 'b0' and wait 4 seconds
        # ('b3', 2),   # Send 'b0.8' and wait 5 seconds
        # ('c3.5', 10),   # Send 'c0.8' and wait 3 seconds
        # ('c0', 10)      # Send 'c0' and wait 6 seconds
    ]
    try:
        while True:
            for command, delay in commands_with_delays:
                a += 1;
                ser.write(f"{command}\n".encode())  # Send command
                print(f"Sent command: {command}")
                print(f"a: {a}")
                time.sleep(delay)  # Wait for the specified delay time
    except KeyboardInterrupt:
        print("Program stopped")
        ser.write(f"{'a0'}\n".encode())
        ser.write(f"{'b0'}\n".encode())
        ser.write(f"{'c0'}\n".encode())
        ser.write(f"{'d0'}\n".encode())
    finally:
        ser.close()  # Close the serial connection