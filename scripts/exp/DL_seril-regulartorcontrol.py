import serial
import time

if __name__ == '__main__':
    port = 'COM12'  # Change this to your port
    baudrate = 115200

    # Open the serial connection
    ser = serial.Serial(port, baudrate, timeout=2)
    try:
        commands = ['a2','b2','c2','d2']
        for command in commands:
            command+='\r\n'
            ser.flushInput()  # Flush input buffer
            time.sleep(0.5)
            ser.flushOutput()  #
            time.sleep(0.5)
            ser.write(command.encode('utf-8'))
            print(f'Sent: {command.strip()}')
            time.sleep(2)  # Wait for 2 seconds



        commands1 = ['d2','a0','c2','d0','b2','c0','a2','b0']


        while True:
            for command in commands1:
                # ser.flushInput()  # Flush input buffer
                # time.sleep(0.5)
                ser.flushOutput()  #
                command += '\r\n'
                ser.write(command.encode('utf-8'))  # Send command
                print(f'Sent: {command.strip()}')
                time.sleep(1)  # Wait for 2 seconds
    except KeyboardInterrupt:
        commands = ['a', 'b', 'c', 'd']
        for command in commands:
            command += '\r\n'
            ser.flushInput()  # Flush input buffer
            time.sleep(0.5)
            ser.flushOutput()  #
            time.sleep(0.5)
            ser.write(command.encode('utf-8'))
            print(f'Sent: {command.strip()}')
            time.sleep(1)  # Wait for 2 seconds