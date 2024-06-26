# for ure use the following version  of urx: https://github.com/jkur/python-urx/tree/SW3.5/urx
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  7 10:17:28 2021

@author: dongting
"""
import numpy
import urx
import socket
import time
import datetime
import matplotlib.pyplot as plt
import time
import matplotlib
import matplotlib.pyplot as plt
from numpy import pi

# For this code, really you should refer to urx python package, I just packed some common functions inside, for example, move and rotate around a axis

def Init_ur(ur_port):

    return ur


def move_ur(ur,moving_vector,v,a,wait=False):
    current_pose = ur.get_pose()
    current_pose.pos[:] += moving_vector
    ur.movel(current_pose,vel=v,acc=a,wait=wait)


def rotate_around_h(angle_r):
    # you must set the TCP correctly
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

# Define moving vector according to the table frames
# unit is M


if __name__ == "__main__":

    moving_vector_left = numpy.array((-1,0,0))
    moving_vector_right = numpy.array((1,0,0))
    moving_vector_forward = numpy.array((0,1,0))
    moving_vector_backward = numpy.array((0,-1,0))
    moving_vector_up = numpy.array((0,0,1))
    moving_vector_down = numpy.array((0,0,-1))

    # Define the robot
    ur_port = "192.168.0.110"
    ur = urx.Robot(ur_port,use_rt=True,urFirm=5.9)
    time.sleep(5)

    # Define the TCP and Payload
    tcp = ((0,0,0.30,0,0,0))
    payload_m = 0.1
    payload_location = (0,0,0.15)

    ur.set_tcp(tcp)
    ur.set_payload(payload_m, payload_location)

    depth = 10/1000
    push_distance = 20/1000

    # test moving
    move_ur(ur,moving_vector_down*depth,0.01,1,wait=True)
    move_ur(ur,moving_vector_left*push_distance,0.01,1,wait=True)
    move_ur(ur,moving_vector_up*depth,0.01,1,wait=True)
    move_ur(ur,moving_vector_right*push_distance,0.01,1,wait=True)


    # Test get some data
    print(ur.get_pose())
    print(ur.get_pos())
    print(ur.getj())


    # Test collect some data
    time_a = time.time()
    move_ur(ur,moving_vector_up*depth,0.01,1,wait=False)
    initial_pose = ur.get_pos()[:]
    pos_time = numpy.append(time.time()-time_a,initial_pose)

    try:
        while True:
            time_b = time.time()-time_a
            position_data = ur.get_pos()[:]-initial_pose
            current_data= numpy.append(time_b,position_data)
    except KeyboardInterrupt:
        move_ur(ur,moving_vector_down*-depth,5e-3,0.1,wait=False)
        plt.plot(current_data)


