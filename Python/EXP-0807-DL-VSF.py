# HKJF
import urx
import threading
import time
import csv
import os
import datetime
import socket
import ctypes
import numpy
from numpy import pi
import matplotlib.pyplot as plt

# Import your existing files
from read_ati_class_rdt import atiSensor  # Assuming this is your ATI class file
from control_ur_robot import move_ur,rotate_around_h
from DataLogger import DataLogger


if __name__ == '__main__':
    try:

        ati_ip = "192.168.0.121"
        robot_ip = "192.168.0.110"
        data_logger = DataLogger(ati_ip=ati_ip,robot_ip=robot_ip)

        # Robot moving loop
        # To access the robot, one can use
        ur16 = data_logger.robot

        # Define robot variables
        moving_vector_left = numpy.array((-1, 0, 0))
        moving_vector_right = numpy.array((1, 0, 0))
        moving_vector_forward = numpy.array((0, 1, 0))
        moving_vector_backward = numpy.array((0, -1, 0))
        moving_vector_up = numpy.array((0, 0, 1))
        moving_vector_down = numpy.array((0, 0, -1))
        # tcp = ((0, 0, 0.30, 0, 0, 0))
        # payload_m = 0.1
        # payload_location = (0, 0, 0.15)
        # ur16.set_tcp(tcp)
        # ur16.set_payload(payload_m, payload_location)
        print(f"Current robot location: {ur16.get_pos()}")
        print(f"Current robot joint angle: {numpy.rad2deg(ur16.getj())} ")

        prepare_pose = [-1.717706028615133, -1.8794928989806117, -1.6501024961471558, -1.1833744806102295, 1.5736362934112549, -1.606333080922262]

        while True:
            user_input = input("Use current robot position as starting point? (Y/N): ").strip().upper()
            if user_input == 'Y':
                exp_pose = ur16.getj()  # Update if user says yes
                break
            elif user_input == 'N':
                exp_pose = prepare_pose
                data_logger.robot.movej(exp_pose, vel=10 / 1000, acc=0.5, wait=True)
                break
            else:
                print("Invalid input. Please enter Y or N.")

        # move_ur(ur16, moving_vector_down*80/1000, 3 / 1000, 1, wait=True)

        # start recording
        data_logger.start_recording()

        data_logger.force_controlled_intrusion(intrusion_threshold=0.5)
        move_ur(ur16, moving_vector_down * 60/1000, 3/1000, 1, wait=True)

        # rotate_around z
        print("Start dragging")
        # rotate_around_h(ur16,(0,0,(-179)*pi/180))
        repeat_time = 4
        for jj in range(repeat_time):
            print(f"Dragging round #{jj+1}/{repeat_time}")
            move_ur(ur16, moving_vector_right * 100 / 1000, 3 / 1000, 1, wait=True)
            time.sleep(1)
            move_ur(ur16, moving_vector_left * 100 / 1000, 3 / 1000, 1, wait=True)
            time.sleep(1)

        time.sleep(3)
        data_logger.save_data()

        data_logger.robot.movej(exp_pose, vel=3 / 1000, acc=0.5, wait=True)
        data_logger.stop_logging()


    except Exception as e:
        print(f"An error occurred: {e}")