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


ati_port = "192.168.0.121"
ur5_port = "192.168.0.110"

moving_vector_left = numpy.array((-1, 0, 0))
moving_vector_right = numpy.array((1, 0, 0))
moving_vector_forward = numpy.array((0, 1, 0))
moving_vector_back = numpy.array((0, -1, 0))
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

bivalve_j = [-1.3118417898761194, -1.5478468940458079, -2.150613784790039, -1.0147336286357422, 1.5736243724822998, 2.679744005203247]

# tensile_j = [-2.7783332506762903, -2.040734430352682, -1.7743191719055176, -0.9084790509990235, 1.5741095542907715, 0.3636760711669922]
# inflated_tensile_j = [-2.778273646031515, -2.0281855068602503, -1.7625762224197388, -0.932675914173462, 1.574264407157898, 0.36365222930908203]
# deflated_tensile_j = [-2.7782819906817835, -2.0379463634886683, -1.7716891765594482, -0.913682298069336, 1.574294924736023, 0.363668829202652]

ur5.movej(bivalve_j,acc=0.1,vel=0.05,wait=True)
time.sleep(2)
calib_data = Calibrate_Ati_Sensor(ati_gamma, ati_port, message)

depth = 100/1000
distance = 100/1000
angle = 0

data_storage = []
time_a = time.time()
initial_pose = ur5.get_pos()[:]

ii = 0
total = 20000

moving_digit = 2

move_ur5(ur5, moving_vector_back * depth, 2e-3, 0.1, wait=False)
while abs(ur5.get_pos()[moving_digit] - initial_pose[moving_digit])< depth*0.99:
    ii += 1
    time_b = time.time() - time_a
    ft_data = get_data(ati_gamma, message) - calib_data
    position_data = ur5.get_pos()[:] - initial_pose
    current_data= []
    current_data0 = numpy.append(time_b, ft_data)
    current_data = numpy.append(current_data0, position_data)

    if ii == 1:  # Initialize data_storage on the first iteration
        data_storage = current_data
    else:
        data_storage = numpy.vstack((data_storage, current_data))

print(ii)
print(data_storage.shape)
move_ur5(ur5, moving_vector_forward * depth, 2e-3, 0.1, wait=False)

# while abs(ur5.get_pos()[moving_digit] - initial_pose[moving_digit])> 0.1/1000:
#     ii += 1
#     time_b = time.time() - time_a
#     ft_data = get_data(ati_gamma, message) - calib_data
#     position_data = ur5.get_pos()[:] - initial_pose
#     current_data= []
#     current_data0 = numpy.append(time_b, ft_data)
#     current_data = numpy.append(current_data0, position_data)
#
#     data_storage = numpy.vstack((data_storage, current_data))
# print(ii)
# print(data_storage.shape)
# # time.sleep(0.0001)
# print((len(data_storage)/data_storage[-1,0]-data_storage[0,0]))

# plt.figure()
# digit = 2
# plt.plot(data_storage[:, 0], data_storage[:, digit])
# plt.ion()
# plt.show()
#
# plt.figure()
# plt.plot(data_storage[:, 9], data_storage[:, 3])

numpy.savetxt("baseline.csv",data_storage, fmt='%.18e', delimiter=',',newline='\n')