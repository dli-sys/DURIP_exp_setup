import numpy as np
import time
import socket
import datetime
import matplotlib.pyplot as plt
import urx
from math import pi
from scipy.signal import butter, filtfilt


class UR_Controller:
    def __init__(self, ur5_ip, ati_ip):
        self.ur5_ip = ur5_ip
        self.ati_ip = ati_ip
        self.TCP_PORT = 49152
        self.BUFFER_SIZE = 1024
        self.order = 'big'
        self.counts_per_unit = np.array([1e6] * 6)
        self.ur5 = None
        self.ati_socket = None
        self.ati_message = None
        self.calib_data = None

    def connect_ur5(self):
        self.ur5 = urx.Robot(self.ur5_ip)
        print("Connected to UR5")

    def connect_ati_sensor(self):
        print("Initializing ATI sensor at", self.ati_ip)
        self.ati_message = (0x1234).to_bytes(2, byteorder=self.order, signed=False)
        self.ati_message += (2).to_bytes(2, self.order)
        self.ati_message += (1).to_bytes(4, self.order)

        self.ati_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ati_socket.settimeout(2)
        self.ati_socket.connect((self.ati_ip, self.TCP_PORT))
        print("ATI Sensor connected")

    def extract_raw(self, packet):
        return [int.from_bytes(packet[12 + i*4:16 + i*4], byteorder=self.order, signed=True) for i in range(6)]

    def calibrate_ati_sensor(self, samples=3000):
        self.ati_socket.connect((self.ati_ip, self.TCP_PORT))
        self.calib_data = np.zeros(6)
        for _ in range(samples):
            self.ati_socket.send(self.ati_message)
            data = self.ati_socket.recv(self.BUFFER_SIZE)
            self.calib_data += np.array(self.extract_raw(data)) / self.counts_per_unit
        self.calib_data /= samples
        print("ATI Calibration complete")

    def get_ati_data(self):
        self.ati_socket.send(self.ati_message)
        data = self.ati_socket.recv(self.BUFFER_SIZE)
        return np.array(self.extract_raw(data)) / self.counts_per_unit

    def move_ur5(self, moving_vector, v, a, wait=False):
        current_pose = self.ur5.get_pose()
        current_pose.pos[:] += moving_vector
        self.ur5.movel(current_pose, vel=v, acc=a, wait=wait)

    def collect_one_line(self, time_ref, data):
        ft = self.get_ati_data() - self.calib_data
        pos = self.ur5.get_pos()[:]
        t = time.time() - time_ref
        line = np.concatenate(([t], ft, pos, [int(self.ur5.is_program_running())]))
        return line if len(data) == 0 else np.vstack((data, line))

    def tictoc(self, data, time_ref, pause=3):
        t0 = time.time()
        while time.time() - t0 < pause:
            data = self.collect_one_line(time_ref, data)
        return data

    def move_with_data(self, data, vector, time_ref, vel=0.01, acc=0.1):
        start = self.ur5.get_pos()[:]
        dist = np.linalg.norm(vector)
        self.move_ur5(vector, v=vel, a=acc, wait=False)
        while np.linalg.norm(self.ur5.get_pos()[:] - start) < 0.995 * dist:
            data = self.collect_one_line(time_ref, data)
        return data

    def rotate_tool(self, angle_rad, axis="rz"):
        import math3d as m3d
        pose = self.ur5.get_pose()
        T = m3d.Transform()
        T.pos = m3d.Vector(0, 0, 0)
        if axis == "rx":
            T.orient = m3d.Orientation.new_euler((angle_rad, 0, 0), 'XYZ')
        elif axis == "ry":
            T.orient = m3d.Orientation.new_euler((0, angle_rad, 0), 'XYZ')
        elif axis == "rz":
            T.orient = m3d.Orientation.new_euler((0, 0, angle_rad), 'XYZ')
        else:
            raise ValueError("Invalid axis")
        self.ur5.movel(pose * T, vel=0.03, acc=1, wait=True, threshold=5)

    def set_tool_io(self, on, pin=8):
        self.ur5.set_tool_voltage(24 if on else 0)
        self.ur5.set_digital_out(pin, int(on))

    def filter_data(self, raw, fs, cutoff=10, order=3):
        b, a = butter(order, cutoff / (fs / 2), btype='low')
        return filtfilt(b, a, raw)

    def save_data(self, data, prefix="exp", aoa=0):
        now = datetime.datetime.now()
        filename = now.strftime(f"%Y-%m-%d-%H-%M_{prefix}_aoa_{aoa}.csv")
        np.savetxt(filename, data, fmt='%.18e', delimiter=',')
        print("Data saved to", filename)

    def plot_force(self, data):
        ts = data[:, 0]
        fs = 100 / (ts[100] - ts[0])
        fx = self.filter_data(data[:, 1], fs)
        fy = self.filter_data(data[:, 2], fs)
        fz = self.filter_data(data[:, 3], fs)
        plt.figure()
        plt.plot(data[:, -3], fx, label='Fx')
        plt.plot(data[:, -3], fy, label='Fy')
        plt.plot(data[:, -3], fz, label='Fz')
        plt.legend()
        plt.grid()
        plt.show()

    def plot_all(self, data):
        for i in range(data.shape[1]):
            plt.figure()
            plt.plot(data[:, 0], data[:, i] if i else data[:, i])
            plt.title(f"Column {i}")
            plt.grid()
            plt.show()