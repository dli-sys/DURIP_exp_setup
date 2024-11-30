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
import math3d as m3d


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
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

def rotate_around_h(ur5,angle_r,rot_vel = 0.05, rot_acc = 1, wait = False):
    # you must set the TCP correctly
    # this code is designed so that rotate the x axis of the TCP make sure tcp is correctly configureed
    # if rotate around the base flange, set tcp as zeros
    # rotate around an axis
    # Tct.orient = m3d.Orientation.new_euler((pi/2,0,0), encoding='XYZ')
    # rotate around y-axis
    # Tct.orient = m3d.Orientation.new_euler((0,pi/2,0), encoding='XYZ')
    # rotate around z-axis
    # Tct.orient = m3d.Orientation.new_euler((0,0,pi/2), encoding='XYZ')
    import math3d as m3d
    pose = ur5.get_pose()
    Tct = m3d.Transform()
    Tct.pos = m3d.Vector(0,0,0)
    Tct.orient = m3d.Orientation.new_euler(angle_r, encoding='XYZ')
    new_pos = pose*Tct
    ur5.movel(new_pos,vel=rot_vel,acc=rot_acc,wait=wait,threshold=5)

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

calib_data = Calibrate_Ati_Sensor(ati_gamma, ati_port, message)
ur5.getj()
sandbox_pos = [-1.7388599554644983, -1.7977224789061488, -1.728344440460205, -1.1867308181575318, 1.5739574432373047, -0.694434944783346]

flapping_joint = [-2.853365484868185, -1.682418008843893, -2.0033633708953857, -1.0275273484042664, 1.5698227882385254, 1.9594440460205078]


while True:
    user_input = input("Use current robot position as starting point? (Y/N): ").strip().upper()
    if user_input == 'Y':
        exp_pose = ur5.getj()  # Update if user says yes
        break
    elif user_input == 'N':
        ur5.movej(sandbox_pos, vel=200/ 1000, acc=0.5, wait=True)
        break
    else:
        print("Invalid input. Please enter Y or N.")

calib_data = Calibrate_Ati_Sensor(ati_gamma, ati_port, message)

depth = 10/1000
distance = 100/1000

move_ur5(ur5,moving_vector_up*100/1000,0.05,1,True)

# rotate_around_h(ur5, [0, 0, pi / 2],rot_vel=0.1,wait=True)
# rotate_around_h(ur5, [0, 0, -pi / 2],rot_vel=0.1,wait=True)

data_storage = []
time_a = time.time()
initial_pose = ur5.get_pos()[:]
initial_joint_angle = ur5.getj()[-1]
ii = 0
angle = 120/180*pi

rot_velocity = 0.1
reptead_time = 5
for rr in range(reptead_time):
    rotate_around_h(ur5, [0, 0, angle],rot_vel=rot_velocity,wait=False)

    while abs(ur5.getj()[-1] - initial_joint_angle)< (angle-pi/180):
        ii += 1
        time_b = time.time() - time_a
        ft_data = get_data(ati_gamma, message) - calib_data
        position_data = ur5.get_pos()[:] - initial_pose
        current_data= []
        current_data0 = numpy.append(time_b, ft_data)
        current_data1 = numpy.append(current_data0, position_data)
        current_data = numpy.append(current_data1, ur5.getj()[-1])

        if ii == 1:  # Initialize data_storage on the first iteration
            data_storage = current_data
        else:
            data_storage = numpy.vstack((data_storage, current_data))

    rotate_around_h(ur5, [0, 0, -angle],rot_vel=rot_velocity,wait=False)
    while abs(ur5.getj()[-1] - initial_joint_angle)> pi/180:
        ii += 1
        time_b = time.time() - time_a
        ft_data = get_data(ati_gamma, message) - calib_data
        position_data = ur5.get_pos()[:] - initial_pose
        current_data= []
        current_data0 = numpy.append(time_b, ft_data)
        current_data1 = numpy.append(current_data0, position_data)
        current_data = numpy.append(current_data1, ur5.getj()[-1])
        data_storage = numpy.vstack((data_storage, current_data))

    print(ii)
    print(data_storage.shape)


initial_angle = -pi/3

print((len(data_storage)/data_storage[-1,0]-data_storage[0,0]))

data_storage_mod = data_storage
offset_angle = -pi/3
data_storage_mod[:,-1] = data_storage[:,-1] - data_storage[0,-1] + offset_angle

plt.close('all')

rotation_matrices = numpy.array([
    [[numpy.cos(angle), -numpy.sin(angle), 0],
     [numpy.sin(angle), numpy.cos(angle), 0],
     [0, 0, 1]]
    for angle in data_storage[:,-1]
])

# Project the force vectors to the world frame
force_vectors_local = numpy.column_stack((data_storage_mod[:,1], data_storage_mod[:,2], data_storage_mod[:,3]))
force_vectors_world = numpy.einsum('ijk,ik->ij', rotation_matrices, force_vectors_local)

# Extract the world frame force components
force_x_world = force_vectors_world[:, 0]
force_y_world = force_vectors_world[:, 1]
force_z_world = force_vectors_world[:, 2]

plt.figure()
plt.plot(force_x_world - force_x_world[0])
plt.grid()
# plt.plot(force_y_world - force_y_world[0])
# plt.plot(force_z_world - force_z_world[0])
plt.show()

plt.figure()
y_max = numpy.max(abs(force_y_world - force_y_world[0]))
plt.plot(numpy.rad2deg(data_storage_mod[:, -1]),force_x_world - force_x_world[0])
plt.plot(numpy.rad2deg(data_storage_mod[:, -1]),force_y_world - force_y_world[0])
# plt.plot(numpy.rad2deg(data_storage_mod[:, -1]),force_z_world - force_z_world[0])
plt.grid()
plt.ylim(-1.2*y_max,1.2*y_max)
plt.show()

fig, (time_based,angle_based) = plt.subplots(1, 2)
digit = 6
time_based.plot(data_storage[:, 0], data_storage[:, digit])
angle_based.plot(numpy.rad2deg(data_storage[:, -1]), data_storage[:, digit])

numpy.savetxt("sand_test_120deg_0907.csv",data_storage, fmt='%.18e', delimiter=',',newline='\n')