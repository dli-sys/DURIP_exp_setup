# for ure use the following version  of urx: https://github.com/jkur/python-urx/tree/SW3.5/urx
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  7 10:17:28 2021

@author: dongting
"""
import numpy
import socket
import time
import os
import datetime
import matplotlib.pyplot as plt
import urx
import serial
import math
import math3d as m3d
import yaml
import logging
import time
import serial
from mpl_toolkits import mplot3d
import matplotlib
import matplotlib.pyplot as plt
from numpy import pi

import numpy as np
from scipy.signal import butter,filtfilt


# For this code, really you should refer to urx python package, I just packed some common functions inside, for example, move and rotate around a axis

def Init_ur(ur_port):

    return ur

def move_ur(ur,moving_vector,v,a,wait=False):
    current_pose = ur.get_pose()
    current_pose.pos[:] += moving_vector
    ur.movel(current_pose,vel=v,acc=a,wait=wait)


def rotate_around_h(angle_r): 
    # this code is designed so that rotate the x axis of the TCP make sure tcp is correctly configureed
    # if rotate around the base flange, set tcp as zeros
    # rotate around a axis
    # Tct.orient = m3d.Orientation.new_euler((pi/2,0,0), encoding='XYZ')
    # rotate around y-axis
    # Tct.orient = m3d.Orientation.new_euler((0,pi/2,0), encoding='XYZ')
    # rotate around z-axis
    # Tct.orient = m3d.Orientation.new_euler((0,0,pi/2), encoding='XYZ')
    import math3d as m3d
    pose = ur.get_pose()
    Tct = m3d.Transform()
    Tct.pos = m3d.Vector(0,0,0)
    Tct.orient = m3d.Orientation.new_euler(angle_r, encoding='XYZ')
    new_pos = pose*Tct
    ur.movel(new_pos,vel=0.03,acc=1,wait=True,threshold=5)

    

moving_vector_left = numpy.array((1,0,0))
moving_vector_right = numpy.array((-1,0,0))
moving_vector_forward = numpy.array((0,-1,0))
moving_vector_backward = numpy.array((0,1,0))
moving_vector_up = numpy.array((0,0,1))
moving_vector_down = numpy.array((0,0,-1))
    

tcp = ((0,0,0.1476,0,0,0))
payload_m = 0.61
payload_location = (0,0,0)    
ur = urx.Robot(ur_port)
time.sleep(5)
ur.set_tcp(tcp)
ur.set_payload(payload_m, payload_location)


# time_a = time.time()

# move_ur(ur,moving_vector_down*depth,0.05,1,wait=False)
# initial_pose = ur.get_pos()[:]
# pos_time = numpy.append(time.time()-time_a,initial_pose)

# try:
#     while True:
#         time_b = time.time()-time_a
#         position_data = ur.get_pos()[:]-initial_pose
#         current_data= numpy.append(time_b,position_data)
# except KeyboardInterrupt:
#     move_ur(ur,moving_vector_down*-depth,5e-3,0.1,wait=False)
#     plt.plot()


pose_90 = [-0.75*pi,-pi/2,pi/2,-pi/2,-pi/2,55/36*pi]
ur.movej(pose_90,vel=0.08,acc=1,wait=True,threshold=5)

# rotate around the h axis


# joint level control to move the robot
# pose_90 = ur.getj()


#init robot pose
# demo_angle  = numpy.array([pi/2,0,0])
# print("Initial Joint:{}".format(numpy.rad2deg(ur.getj())))
# print("Initial Loc:{}".format([ur.get_pos()*1000]))
# rotate_around_h(demo_angle)
# print("Middle Loc:{}".format([ur.get_pos()*1000]))
# print("Middle Joint:{}".format(numpy.rad2deg(ur.getj())))
# rotate_around_h(-1*demo_angle)
# print("End Loc:{}".format([ur.get_pos()*1000]))
# print("End Joint:{}".format(numpy.rad2deg(ur.getj())))

# rorate in z-axis
demo_angle  = numpy.array([0,0,-pi/6])
rotate_around_h(demo_angle)

# exp_pos = [-0.13606, 0.50324, 0.50298]
# ur.set_pos(exp_pos,vel=0.05,acc=1,wait=False,threshold=5)