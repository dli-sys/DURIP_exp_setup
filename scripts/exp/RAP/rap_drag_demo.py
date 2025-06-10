# -*- coding: utf-8 -*-
"""
UR5e + ATI Gamma Drag Test Script with Fluidization Control
@author: dongting
Refactored by ChatGPT
"""

import numpy as np
import time
import matplotlib.pyplot as plt
import urx
import socket
import collections
import collections.abc
collections.Iterable = collections.abc.Iterable
from scipy.signal import butter, filtfilt

# Constants
TCP_PORT = 49152
BUFFER_SIZE = 1024
ORDER = 'big'
COUNTS_PER_UNIT = np.array([1e6] * 6)

def Init_ur5(ur5_ip):
    try:
        robot = urx.Robot(ur5_ip)
        print("Connected to UR5")
        return robot
    except:
        print("Failed to connect to UR5.")
        return None

def move_ur5(robot, move_vec, vel, acc, wait=False):
    pose = robot.get_pose()
    pose.pos[:] += move_vec
    try:
        robot.movel(pose, vel=vel, acc=acc, wait=wait)
    except urx.urrobot.RobotException as e:
        print("Caught RobotException: trying again...")
        time.sleep(0.5)
        try:
            if robot.is_program_running():
                robot.movel(pose, vel=vel, acc=acc, wait=wait)
            else:
                print("Robot program not running, attempting movej fallback.")
                joints = robot.getj()
                robot.movej(joints, vel=vel, acc=acc, wait=wait)
        except Exception as e:
            raise RuntimeError(f"Move failed again: {e}")

def Init_Ati_Sensor(ip):
    print(f"Initializing ATI sensor at {ip}")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(2)
    s.connect((ip, TCP_PORT))
    message = (0x1234).to_bytes(2, ORDER) + (2).to_bytes(2, ORDER) + (1).to_bytes(4, ORDER)
    print("ATI sensor connected")
    return s, message

def extract_raw(packet):
    return [int.from_bytes(packet[12+i*4:16+i*4], byteorder=ORDER, signed=True) for i in range(6)]

def Calibrate_Ati_Sensor(sock, ip, msg, samples=3000):
    print("Calibrating ATI sensor in air...")
    calib_data = np.zeros(6)
    for _ in range(samples):
        sock.send(msg)
        packet = sock.recv(BUFFER_SIZE)
        raw = np.array(extract_raw(packet))
        calib_data += raw / COUNTS_PER_UNIT
    return calib_data / samples

def get_data(sock, msg):
    sock.send(msg)
    packet = sock.recv(BUFFER_SIZE)
    raw = np.array(extract_raw(packet))
    return raw / COUNTS_PER_UNIT

def log_during_duration(robot, sock, msg, calib, initial_pose, duration, data_log):
    t_start = time.time()
    while time.time() - t_start < duration:
        t_now = time.time() - data_log[0, 1] if data_log.size else 0.0
        pos = robot.get_pos() - initial_pose
        ft = get_data(sock, msg) - calib
        new_row = [len(data_log), t_now] + list(pos) + list(ft)
        data_log = np.vstack([data_log, new_row]) if data_log.size else np.array([new_row])
    return data_log

def move_ur5_w_collect(robot, sock, msg, calib, data, move_vec, vel, acc, initial_pose, data_log):
    start_pose = robot.get_pos()
    total_dist = np.linalg.norm(move_vec)
    move_ur5(robot, move_vec, vel=vel, acc=acc, wait=False)
    t_start = time.time()

    while np.linalg.norm(robot.get_pos() - start_pose) < total_dist * 0.995:
        t_now = time.time() - global_start
        ft = get_data(sock, msg) - calib
        pos_diff = robot.get_pos() - initial_pose
        new_row = [len(data), t_now] + list(pos_diff) + list(ft)
        data = np.vstack([data, new_row]) if data.size else np.array([new_row])
    return data

if __name__ == '__main__':
    global_start = time.time()
    # Config
    ati_ip = "192.168.0.121"
    ur5_ip = "192.168.0.110"
    repeat_time = 1
    move_distance = 0.1  # meters
    velocity = 0.02       # m/s
    acceleration = 0.1
    down_depth = 0.01     # meters
    fluidization_pin = 4

    vecs = {
        'left': np.array([-1, 0, 0]),
        'right': np.array([1, 0, 0]),
        'forward': np.array([0, 1, 0]),
        'backward': np.array([0, -1, 0]),
        'up': np.array([0, 0, 1]),
        'down': np.array([0, 0, -1])
    }

    sock, msg = Init_Ati_Sensor(ati_ip)
    calib_data = Calibrate_Ati_Sensor(sock, ati_ip, msg)

    ur5 = Init_ur5(ur5_ip)
    if ur5 is None:
        exit()

    if input("Use current robot position? (y/n): ").strip().lower() != 'y':
        prepare_j = [-1.5871, -1.7489, -2.0817, -0.8993, 1.5704, 2.4225]
        ur5.movej(prepare_j, vel=0.03, acc=1, wait=True)

    input("Start fluid, press Enter to continue")

    print("Skipping recalibration — using initial air calibration.")
    initial_pose = ur5.get_pos()
    ft = get_data(sock, msg) - calib_data
    pos = ur5.get_pos() - initial_pose
    initial_row = [0, 0.0] + list(pos) + list(ft)
    data_log = np.array([initial_row])

    print("Activating fluidization (DO4 HIGH)...")
    ur5.set_digital_out(fluidization_pin, 1)
    data_log = log_during_duration(ur5, sock, msg, calib_data, initial_pose, duration=2, data_log=data_log)

    print(f"Inserting object to depth {down_depth*1000:.1f} mm...")
    data_log = move_ur5_w_collect(ur5, sock, msg, calib_data, data_log, vecs['down'] * down_depth, vel=0.02, acc=1, initial_pose=initial_pose,data_log=data_log)


    print("Holding position — logging will continue.")

    print("Deactivating fluidization (DO4 LOW) and letting system settle...")
    ur5.set_digital_out(fluidization_pin, 0)
    data_log = log_during_duration(ur5, sock, msg, calib_data, initial_pose, duration=3, data_log=data_log)

    for _ in range(repeat_time):
        data_log = move_ur5_w_collect(ur5, sock, msg, calib_data, data_log,
                                      vecs['right'] * move_distance, velocity, acceleration, initial_pose,data_log=data_log)
        data_log = move_ur5_w_collect(ur5, sock, msg, calib_data, data_log,
                                      vecs['left'] * move_distance, velocity, acceleration, initial_pose,data_log=data_log)

    print(f"Sampling rate: {len(data_log)/(data_log[-1,1]-data_log[0,1]):.2f} Hz")

    plt.figure()
    plt.plot(data_log[:, 3]*1e3, data_log[:, 4])
    plt.title("Fx vs X Position (mm)")

    plt.figure()
    plt.plot(data_log[:, 3]*1e3, data_log[:, 5])
    plt.title("Fy vs X Position (mm)")

    plt.figure()
    plt.plot(data_log[:, 3]*1e3, data_log[:, 6])
    plt.title("Fz vs X Position (mm)")

    plt.show(block=True)

    from datetime import datetime
    timestamp = datetime.now().strftime("%H-%M-%S")
    filename = f"rap_drag_exp_{timestamp}.csv"
    np.savetxt(filename, data_log, fmt='%.18e', delimiter=',')

    plt.figure()
    for ii in range(data_log.shape[1]):
        plt.plot(data_log[:, ii])
        plt.show(block=True)
