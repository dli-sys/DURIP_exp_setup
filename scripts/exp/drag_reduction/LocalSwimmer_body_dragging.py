# for ur5e use the following version  of urx: https://github.com/jkur/python-urx/tree/SW3.5/urx
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  7 10:17:28 2021

@author: dongting
"""
import numpy

import time
import datetime
import matplotlib.pyplot as plt
import urx
import time
import matplotlib.pyplot as plt
from math import pi
import socket


def Init_ur5(ur5_port):
    try:
        # tcp = ((0, 0, 0, 0, 0, 0))
        # payload_m = 1
        # payload_location = (0, 0, 0.5)
        ur5 = urx.Robot(ur5_port)
        # ur5.set_tcp(tcp)
        # ur5.set_payload(payload_m, payload_location)
        print("Connected to Ur5")
    except:
        print("Can not connect, check connection and try again")
        ur5 = None
    return ur5


def move_ur5(ur5, moving_vector, v, a, wait=False):
    current_pose = ur5.get_pose()
    current_pose.pos[:] += moving_vector
    ur5.movel(current_pose, vel=v, acc=a, wait=wait)


def rotate_ur(ur5, angle_r, axis="rz", warning=True):
    """
    Rotates the robot around the specified axis (x, y, z, or custom (rx, ry, rz)).

    :param angle_r: The rotation angle in radians (single value for axis rotations, tuple for custom rotations).
    :param axis: The axis around which to rotate ("x", "y", "z", or "custom" for (rx, ry, rz)).
    :param warning: If True, show a warning before proceeding with rotation.
    """
    import math3d as m3d

    # Warning mechanism
    if warning:
        print("Warning: Using this function may damage cables or other equipment. Ensure everything is clear.")
        confirmation = input("Type 'CONFIRM' to proceed with rotation or anything else to cancel: ")
        if confirmation != "CONFIRM":
            raise ValueError("Operation canceled. You must confirm by typing 'CONFIRM' to proceed.")

    # Proceed with rotation after confirmation
    pose = ur5.get_pose()
    Tct = m3d.Transform()
    Tct.pos = m3d.Vector(0, 0, 0)  # No translation, just rotation

    if axis == "rx":
        # Rotation around x-axis
        Tct.orient = m3d.Orientation.new_euler((angle_r, 0, 0), encoding='XYZ')
    elif axis == "ry":
        # Rotation around y-axis
        Tct.orient = m3d.Orientation.new_euler((0, angle_r, 0), encoding='XYZ')
    elif axis == "rz":
        # Rotation around z-axis
        Tct.orient = m3d.Orientation.new_euler((0, 0, angle_r), encoding='XYZ')
    elif axis == "custom" and isinstance(angle_r, tuple) and len(angle_r) == 3:
        # Custom rotation (rx, ry, rz)
        rx, ry, rz = angle_r
        Tct.orient = m3d.Orientation.new_euler((rx, ry, rz), encoding='XYZ')
    else:
        raise ValueError("Invalid axis or custom angle format. Use 'x', 'y', 'z' or 'custom' with (rx, ry, rz).")

    new_pos = pose * Tct  # Apply the rotation transformation
    ur5.movel(new_pos, vel=0.03, acc=1, wait=True, threshold=5)


def Init_Ati_Sensor(TCP_IP):
    import socket
    print("Initilizing ati sensor")
    # global TCP_IP, TCP_PORT, BUFFER_SIZE, order

    print("Start connection to " + TCP_IP)
    # global message
    message = b''
    message += (0x1234).to_bytes(2, byteorder=order, signed=False)
    message += (2).to_bytes(2, order)
    message += (1).to_bytes(4, order)
    #    message = b'\x124\x00\x02\x00\x00\x00\x01'
    # global s
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(2)
    s.connect((TCP_IP, TCP_PORT))
    print("Sensor connected")
    return s, message


def extract_raw(packet):
    import socket
    raw = []
    for ii in range(6):
        byte = packet[12 + ii * 4:12 + ii * 4 + 4]
        value = int.from_bytes(byte, byteorder=order, signed=True)
        raw.append(value)
    return raw


def extract_scaling(packet):
    import socket
    raw = []
    for ii in range(6):
        byte = packet[ii + 2:ii + 4]
        value = int.from_bytes(byte, byteorder=order, signed=False)
        raw.append(value)
    return raw


def Calibrate_Ati_Sensor(s, TCP_IP, message):
    import socket
    import numpy as np
    s.connect((TCP_IP, TCP_PORT))
    # global calib_data
    calib_data = np.zeros([1, 6])
    for j in range(0, 3000):
        s.send(message)
        data = s.recv(BUFFER_SIZE)
        data2 = np.array(extract_raw(data))
        scaled_data = data2 / counts_per_unit
        calib_data = calib_data + scaled_data
    calib_data = calib_data / 3000
    return calib_data


def get_data(s, message):
    import numpy as np
    import socket
    s.send(message)
    data = s.recv(BUFFER_SIZE)
    data2 = np.array(extract_raw(data))
    ati_data = data2 / counts_per_unit
    return ati_data


def butter_lowpass_filter(data, cutoff, fs, order):
    from scipy.signal import butter, filtfilt
    normal_cutoff = cutoff / (fs/2)
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y


def collect_oneline(calib_data, time_a, existing_data):
    ft_data = get_data(ati_gamma, message) - calib_data
    position_data = ur5.get_pos()[:]
    time_b = time.time() - time_a
    ini_line1 = numpy.append(time_b, ft_data)
    new_line1 = numpy.append(ini_line1, position_data)
    new_line = numpy.append(new_line1, int(ur5.is_program_running()))
    # time.sleep(0.)
    if len(existing_data) == 0:
        existing_data_1 =  new_line  # Create a new list of lists
    else:
        existing_data_1 = numpy.vstack((existing_data, new_line))
    # existing_data.append(new_line)  # Append to the existing list of lists
    return existing_data_1

def move_ur5_w_collect(ur5, ext_data, moving_vector, time_a,vel=1e-3, acc=0.1):
    # ext_data = data_storage
    # moving_vector =moving_vector_backward*distance
    # vel = 5/1000
    # acc=1
    int_initial_pose = ur5.get_pos()[:]
    # time_a = time.time()
    total_distance = numpy.linalg.norm(moving_vector)
    move_ur5(ur5, moving_vector, v=vel, a=acc, wait=False)
    while abs(numpy.linalg.norm(ur5.get_pos()[:] - int_initial_pose)) < total_distance * 0.995:
        ext_data = collect_oneline(calib_data, time_a, ext_data)
    return ext_data  # will python operate this variable?


def tictoc(data_storage,time_a,pause_time = 3):
    time_tic = time.time()
    while (time.time() - time_tic) < pause_time:
        data_storage = collect_oneline(calib_data, time_a, data_storage)
    return data_storage

def plot_data(data,key1='ts',key2='Fx'):
    raw_header = ["ts", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "x", "y", "z","isMoving"]
    header = {raw_header: i for i, raw_header in enumerate(raw_header)}
    plt.plot(data[:, header[key1]], data[:, header[key2]])


if __name__ == '__main__':

    ati_port = "192.168.0.121"
    ur5_port = "192.168.0.110"

    moving_vector_left = numpy.array((-1, 0, 0))
    moving_vector_right = numpy.array((1, 0, 0))
    moving_vector_forward = numpy.array((0, 1, 0))
    moving_vector_backward = numpy.array((0, -1, 0))
    moving_vector_up = numpy.array((0, 0, 1))
    moving_vector_down = numpy.array((0, 0, -1))

    counts_per_unit = numpy.array([1000000] * 3 + [1000000] * 3)
    TCP_PORT = 49152
    BUFFER_SIZE = 1024
    order = 'big'

    ati_gamma, message = Init_Ati_Sensor(ati_port)
    calib_data = Calibrate_Ati_Sensor(ati_gamma, ati_port, message)


    ur5 = Init_ur5(ur5_port)

    # Prepare pose is 5 mm above sand suraface and angle of attack is 0 deg-- meaning that has least contact area
    prepare_pose = [-1.9841793219195765, -1.83571257213735, -1.482507348060608, -1.394862489109375, 1.573994517326355, 0.4221869707107544]

    ur5.movej(prepare_pose,vel=50/1000,acc=1,wait=True)


    fluidlization_pin = 4
    ur5.set_digital_out(fluidlization_pin,1)
    time.sleep(5)

    robot_depth = 70/1000
    body_robot_depth = (70+66)/1000
    move_ur5(ur5, moving_vector_down*body_robot_depth, v = 10/1000,a=0.1,wait=True)
    angle_of_attack = 0
    # rotate_ur(ur5, angle_of_attack/180*pi)


    ur5.set_digital_out(fluidlization_pin,0)
    time.sleep(15)
    print("exp")

    calib_data = Calibrate_Ati_Sensor(ati_gamma, ati_port, message)

    time_a = time.time()
    data_storage = []
    initial_pose = ur5.get_pos()[:]
    distance = 150/1000

    time_a = time.time()
    data_storage = []
    data_storage = collect_oneline(calib_data, time_a, data_storage)

    data_storage = tictoc(data_storage, time_a,pause_time=3)
    # Here add the vibration
    input("start_vib")
    time_v = 3 + time_a
    time_a = time_v

    data_storage = tictoc(data_storage, time_a,pause_time=3)

    repeat_time = 5
    for jj in range(repeat_time):
        data_storage = move_ur5_w_collect(ur5, data_storage, moving_vector_backward*distance,time_a, vel = 10/1000, acc = 0.1)
        data_storage = tictoc(data_storage,time_a,pause_time=1)
        data_storage = move_ur5_w_collect(ur5, data_storage, moving_vector_forward* distance, time_a,vel=10/1000, acc = 0.1)
        data_storage = tictoc(data_storage, time_a,pause_time=1)


    plt.close('all')
    plt.figure()
    # # plot_data(data_storage, key1='y', key2='Fx')
    # # plt.plot(data_storage[:,2])
    # plt.show(block=True)

    sampling_rate = 100/( data_storage[100,0] - data_storage[0,0])
    cut_off = 10
    force_data_x = butter_lowpass_filter(data_storage[:,1], cut_off, sampling_rate, 3)
    force_data_y = butter_lowpass_filter(data_storage[:, 2], cut_off, sampling_rate, 3)
    force_data_z = butter_lowpass_filter(data_storage[:, 3], cut_off, sampling_rate, 3)

    plt.plot(data_storage[:, -3],force_data_x)
    plt.plot(data_storage[:, -3],force_data_y)
    plt.plot(data_storage[:, -3],force_data_z)
    plt.show(block=True)


    for ii in range(11):
        if ii ==0:
            plt.plot(data_storage[:, ii])
        else:
            plt.plot(data_storage[:, 0],data_storage[:, ii])
        plt.show(block=True)

    move_ur5(ur5, moving_vector_up*robot_depth, v = 10/1000,a=0.1,wait=True)
    ur5.movej(prepare_pose,vel=50/1000,acc=1,wait=True)


    now = datetime.datetime.now()
    timestamp_str = now.strftime("%Y-%m-%d-%H-%M_")

    # exp_name = "fin_lf"
    exp_name = "body_qs"
    rest_of_exp = exp_name+ "_aoa_" + str(angle_of_attack) +".csv"
    numpy.savetxt(timestamp_str+rest_of_exp,data_storage, fmt='%.18e', delimiter=',',newline='\n')