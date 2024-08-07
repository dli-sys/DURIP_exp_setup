import socket
import numpy
import time
import struct
import matplotlib.pyplot as plt

class atiSensor:
    def __init__(self, tcp_ip='192.168.0.121', port=49152):
        """
        Initialize the atiSensor object and establish a UDP connection.
        """
        self.calib_data = [0, 0, 0, 0, 0, 0]
        self.COUNTS_PER_UNIT = numpy.array([1000000] * 6)  # U
        self.CALIBRATE_NUM = 21000
        self.tcp_ip = tcp_ip
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.settimeout(2)
        try:
            self.s.connect((self.tcp_ip, self.port))
            print("Sensor connected")
            self.start_streaming()
            self._calibrate()8
        except socket.timeout:
            raise ConnectionError(f"Could not connect to the sensor at {self.tcp_ip}.")

    def start_streaming(self):
        """
        Send the command to start high-speed real-time streaming.
        """
        header = 0x1234
        command = 0x0002  # Command for high-speed real-time streaming
        sample_count = 0  # Infinite samples
        message = struct.pack('>HHI', header, command, sample_count)
        self.s.send(message)

    def collect_sample(self):
        """
        Collect a single sample and return data with sequence and force information.
        """
        data, _ = self.s.recvfrom(36)  # 36 bytes for one RDT record
        unpacked_data = struct.unpack('>IIIiiiiii', data)
        scaled_data = unpacked_data[3:9]/self.COUNTS_PER_UNIT - self.calib_data
        return numpy.concatenate((unpacked_data[:3], scaled_data))

    def _calibrate(self):
        """
        Calibrate the sensor by collecting a set number of samples and calculating the offset.
        """
        calib_data_list = []
        for j in range(self.CALIBRATE_NUM):
            calib_data_list.append(self.collect_sample()[3:9])
        calib_data = numpy.array(calib_data_list)
        self.calib_data = numpy.mean(calib_data, axis=0)
        print("Calibration finished, current offset is", self.calib_data)


    def stop_streaming(self):
        """
        Send the command to stop streaming.
        """
        header = 0x1234
        command = 0x0000  # Command to stop streaming
        message = struct.pack('>HHI', header, command, 0)
        self.s.send(message)

if __name__ == '__main__':
    ati_sensor = atiSensor(tcp_ip="192.168.0.121")  # Replace with actual sensor IP
    try:
        num_samples = 7000  # Specify the number of samples to collect
        data_array = numpy.zeros((num_samples, 9))  # 9 data points per sample
        timestamps = numpy.zeros(num_samples)

        for i in range(num_samples):
            start_time = time.time()  # Record the time just before receiving the data
            data_array[i] = ati_sensor.collect_sample()
            timestamps[i] = start_time

        elapsed_time = timestamps[-1] - timestamps[0]
        sampling_rate = (num_samples - 1) / elapsed_time  # Calculate sampling rate

        fig, ax = plt.subplots(1, 2, figsize=(12, 6))
        ax[0].plot(timestamps - timestamps[0], data_array[:, 3:6],label = ["Fx","Fy","Fz"])  # Force data (Fx, Fy, Fz)
        ax[1].plot(timestamps - timestamps[0], data_array[:, 6:9],label = ["Tx","Ty","Tz"])  # Torque data (Tx, Ty, Tz)
        ax[0].set_title('Force over Time')
        ax[1].set_title('Torque over Time')
        ax[0].set_xlabel('Time (s)')
        ax[1].set_xlabel('Time (s)')
        ax[0].set_ylabel('Force (N)')
        ax[1].set_ylabel('Torque (Nm)')
        ax[0].legend()
        ax[1].legend()
        plt.show(block=True)

        print(f"Calculated Sampling Rate: {sampling_rate} samples per second")
    finally:
        ati_sensor.stop_streaming()
