# for ur5e use the following version of urx: https://github.com/jkur/python-urx/tree/SW3.5/urx
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 7, 2021
@author: dongting
"""


import time
import matplotlib.pyplot as plt
import urx
import socket
import numpy


# Initialize UR5 robot connection
def Init_ur5(ur5_port):
    try:
        ur5 = urx.Robot(ur5_port)
        print("Connected to UR5 robot.")
        return ur5
    except Exception as e:
        print(f"Error connecting to UR5: {e}")
        return None


# Move the UR5 robot
def move_ur5(ur5, moving_vector, v, a, wait=False):
    current_pose = ur5.get_pose()
    current_pose.pos[:] += moving_vector
    ur5.movel(current_pose, vel=v, acc=a, wait=wait)


# Initialize ATI sensor connection
def Init_Ati_Sensor(TCP_IP, TCP_PORT=49152, BUFFER_SIZE=1024, order='big'):
    try:
        print(f"Initializing ATI sensor at {TCP_IP}:{TCP_PORT}...")
        message = b''
        message += (0x1234).to_bytes(2, byteorder=order, signed=False)
        message += (2).to_bytes(2, byteorder=order)
        message += (1).to_bytes(4, byteorder=order)

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2)
        s.connect((TCP_IP, TCP_PORT))
        print("Connected to ATI sensor.")
        return s, message
    except socket.error as e:
        print(f"Error connecting to ATI sensor: {e}")
        return None, None


# Extract raw data from ATI sensor packet
def extract_raw(packet, order='big'):
    raw = []
    for ii in range(6):
        byte = packet[12 + ii * 4:12 + ii * 4 + 4]
        value = int.from_bytes(byte, byteorder=order, signed=True)
        raw.append(value)
    return raw


# Calibrate the ATI sensor
def Calibrate_Ati_Sensor(s, message, counts_per_unit, num_samples=3000):
    try:
        calib_data = numpy.zeros(6)
        for _ in range(num_samples):
            s.send(message)
            data = s.recv(1024)
            data_raw = numpy.array(extract_raw(data))
            scaled_data = data_raw / counts_per_unit
            calib_data += scaled_data
        calib_data /= num_samples
        print("Calibration completed successfully.")
        return calib_data
    except Exception as e:
        print(f"Error during ATI sensor calibration: {e}")
        return None


# Get data from ATI sensor
def get_data(s, message):
    import numpy as np
    import socket
    s.send(message)
    data = s.recv(BUFFER_SIZE)
    data2 = np.array(extract_raw(data))
    ati_data = data2 / counts_per_unit
    return ati_data


# Collect one line of data
def collect_oneline(calib_data, time_a, ur5, ati_gamma, message, existing_data=None):
    if existing_data is None:
        existing_data = []
    try:
        ft_data = get_data(ati_gamma, message) - calib_data
        position_data = ur5.get_pos()[:]
        time_b = time.time() - time_a
        new_line = numpy.append([time_b], numpy.append(ft_data, position_data))
        return numpy.vstack((existing_data, new_line)) if len(existing_data) else new_line
    except Exception as e:
        print(f"Error in collecting data: {e}")
        return existing_data


# Move UR5 while collecting data
def move_ur5_w_collect(ur5, ext_data, moving_vector, vel=1e-3, acc=0.1, time_a=None):
    initial_pose = ur5.get_pos()[:]
    total_distance = numpy.linalg.norm(moving_vector)

    move_ur5(ur5, moving_vector, v=vel, a=acc, wait=False)

    while numpy.linalg.norm(ur5.get_pos()[:] - initial_pose[:]) < total_distance * 0.995:
        ext_data = collect_oneline(calib_data, time_a, ur5, ati_gamma, message, ext_data)
    return ext_data


# Collect data for a specific pause time
def tictoc(calib_data, time_a, ur5, ati_gamma, message, data_storage, pause_time=3):
    time_tic = time.time()
    initial_pose = ur5.get_pos()[:]
    while time.time() - time_tic < pause_time:
        data_storage = collect_oneline(calib_data, time_a, ur5, ati_gamma, message, data_storage)
    return data_storage


def plot_data(data,key1='ts',key2='Fx'):
    raw_header = ["ts", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "x", "y", "z","isMoving"]
    header = {raw_header: i for i, raw_header in enumerate(raw_header)}
    plt.plot(data[:, header[key1]], data[:, header[key2]])

# Main function
if __name__ == '__main__':
    ati_port = "192.168.0.121"
    ur5_port = "192.168.0.110"

    counts_per_unit = numpy.array([1000000] * 3 + [1000000] * 3)
    TCP_PORT = 49152
    BUFFER_SIZE = 1024

    moving_vector_left = numpy.array((-1,0,0))
    moving_vector_right = numpy.array((1,0,0))
    moving_vector_forward = numpy.array((0,1,0))
    moving_vector_backward = numpy.array((0,-1,0))
    moving_vector_up = numpy.array((0,0,1))
    moving_vector_down = numpy.array((0,0,-1))


    # Initialize UR5 and ATI sensor
    ur5 = Init_ur5(ur5_port)
    ati_gamma, message = Init_Ati_Sensor(ati_port)

    # Calibrate ATI sensor
    calib_data = Calibrate_Ati_Sensor(ati_gamma, message, counts_per_unit)
    # Start data collection
    try:
        time_a = time.time()
        initial_pose = ur5.get_pos()[:]
        data_storage = []
        # Move robot and collect data
        data_storage = tictoc(calib_data, time_a, ur5, ati_gamma, message, data_storage, pause_time=1)
        repeat_time = 2
        for _ in range(repeat_time):

            data_storage = move_ur5_w_collect(ur5, data_storage, moving_vector_left * 0.1, vel=0.02, acc=0.1, time_a=time_a)
            data_storage = tictoc(calib_data, time_a, ur5, ati_gamma, message, data_storage, pause_time=1)

            data_storage = move_ur5_w_collect(ur5, data_storage, moving_vector_right * 0.1, vel=0.02, acc=0.1, time_a=time_a)
            data_storage = tictoc(calib_data, time_a, ur5, ati_gamma, message, data_storage, pause_time=1)

        #
        # plt.figure()
        # plt.plot(data_storage[:,0])
        # plt.show(block=True)

        plt.close('all')
        plt.figure()
        plot_data(data_storage, key1='ts', key2='Fx')
        # plot_data(data_storage, key1='y', key2='Fy')
        plt.show(block=True)

        # k1 = 0
        # k2 = 3
        # plt.plot(data_storage[:,k1],data_storage[:,k2])
        # plt.xlabel("Distance (mm)")
        # plt.ylabel("Force (N)")
        # plt.title("Drag_force_reduction")
        plt.show(block=True)


        # Save collected data
        numpy.savetxt("data.csv", data_storage, fmt='%.6e', delimiter=',')
        print("Data collection and saving completed.")
    except KeyboardInterrupt:
        print("Operation interrupted by user.")
    finally:
        ur5.close()
