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


class DataLogger:
    def __init__(self, robot_ip=None, ati_ip=None):
        self.use_robot = robot_ip is not None
        self.use_ati = ati_ip is not None

        if not self.use_robot and not self.use_ati:
            raise ValueError("At least one data source (robot or ATI sensor) must be specified.")

        # Initialize robot connection (if needed)
        if self.use_robot:
            try:
                self.robot = urx.Robot(robot_ip, use_rt=True, urFirm=5.9)
                self.robot_data = []
                time.sleep(5)
                self.initial_pose = self.robot.get_pos()
            except urx.urrobot.RobotException as e:
                print(f"Error connecting to robot: {e}")
                raise

        # Initialize ATI sensor (if needed)
        if self.use_ati:
            try:
                self.ati_sensor = atiSensor(tcp_ip=ati_ip)
                self.load_cell_data = []
            except (ConnectionError, socket.timeout) as e:
                print(f"Error connecting to ATI sensor: {e}")
                if self.use_robot:
                    self.robot.close()
                raise

        # Initialize other variables
        self.stop_event = threading.Event()
        self.index = 0
        self.initial_timestamp = time.time()
        # Start threads only for the enabled devices
        if self.use_robot:
            threading.Thread(target=self.log_robot_data, daemon=True).start()
        if self.use_ati:
            threading.Thread(target=self.log_load_cell_data, daemon=True).start()


    def force_controlled_intrusion(self,step = 1/1000,intrusion_threshold=2):
        moving_vector_down = numpy.array((0, 0, -1*step))
        calib_data_z = 0
        while calib_data_z < intrusion_threshold:
            desire_pose = self.robot.get_pose()
            desire_pose.pos[:]+=moving_vector_down
            self.robot.movel(desire_pose, acc=0.01, vel=0.5 / 1000, wait=True)
            calib_data_list = 0
            for j in range(7000):
                calib_data_list += self.load_cell_data[-1][7]
                # print(self.load_cell_data)
            calib_data_z= abs(calib_data_list/3500)
            print(f"Current z reading: {calib_data_z}")

    def robot_pose_calibration(self):
        pass


    def log_robot_data(self):
        while not self.stop_event.is_set():
            try:
                timestamp = time.time() - self.initial_timestamp
                pos_data = self.robot.get_pos() - self.initial_pose
                xyz = self.robot.get_pos()
                rx, ry, rz = self.robot.get_orientation().to_euler('xyz')
                isRunning_flag = int(self.robot.is_program_running())
                self.robot_data.append((timestamp, self.index, *xyz, rx, ry, rz, isRunning_flag))
                self.index += 1

                if self.index % 1000 == 0:
                    self.stop_event.wait(0)
            except urx.urrobot.RobotException as e:
                if "Robot stopped" in str(e):
                    print("Robot stopped unexpectedly. Exiting.")
                else:
                    print(f"Unexpected URX exception: {e}")
                self.stop_logging()
                break
            finally:
                time.sleep(0.005)  # Adjust sleep interval if needed

    def log_load_cell_data(self):
        while not self.stop_event.is_set():
            try:
                timestamp = time.time() - self.initial_timestamp
                data = self.ati_sensor.collect_sample()
                self.load_cell_data.append((timestamp, self.index, *data))
                self.index += 1
            except socket.timeout:
                print("Load cell communication timeout. Exiting.")
                self.stop_logging()
                break
            # time.sleep(0.00001)

    def save_data(self):
        data_folder = "data"
        os.makedirs(data_folder, exist_ok=True)

        # Generate filename with current date and time
        now = datetime.datetime.now()
        timestamp_str = now.strftime("%Y_%m_%d_%H_%M")

        if self.use_robot:
            robot_filename = os.path.join(data_folder, f"{timestamp_str}_UR.csv")
            with open(robot_filename, 'w', newline='') as robot_file:
                robot_writer = csv.writer(robot_file)
                robot_writer.writerow(["Timestamp", "Index","X", "Y", "Z", "Rx", "Ry", "Rz", "isRunning"])
                robot_writer.writerows(self.robot_data)
                print(f"Data saved to {robot_filename} ")

        if self.use_ati:
            load_cell_filename = os.path.join(data_folder, f"{timestamp_str}_FT.csv")
            with open(load_cell_filename, 'w', newline='') as load_cell_file:
                load_cell_writer = csv.writer(load_cell_file)
                load_cell_writer.writerow(["Timestamp", "Index", "RDTSequence", "FTSerialNumber", "Status", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"])
                load_cell_writer.writerows(self.load_cell_data)
                print(f"Data saved to {load_cell_filename} ")


    def stop_logging(self):
        self.stop_event.set()
        time.sleep(1)  # Allow time for threads to stop gracefully

        for thread in threading.enumerate():
            if thread.name != "MainThread":
                thread_id = thread.ident
                res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, ctypes.py_object(SystemExit))
                if res > 1:
                    ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
                    print('Exception raise failure')
        self.save_data()


if __name__ == '__main__':


    try:

        ati_ip = "192.168.0.121"
        robot_ip = "192.168.0.110"
        data_logger = DataLogger(ati_ip=ati_ip,robot_ip=robot_ip)
        # robot_ip = "192.168.0.110"
        # data_logger = DataLogger(robot_ip=args.robot_ip, ati_ip=args.ati_ip)

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
        print(ur16.get_pos())
        print(ur16.getj())

        # move to prepare pose that you can move the box 0731 (initial pose)
        # prepare_pose = [-1.5899317900287073, -1.8535644016661585, -1.9331345558166504, -0.926342324619629, 1.573030948638916, 0.7853751182556152]
        prepare_pose = [-1.589989964162008, -1.8645616970457972, -1.9449468851089478, -0.9032913011363526, 1.5730645656585693, 0.7853822708129883]


        fluidize_pose = [-1.5892613569842737, -1.7400156460204066, -1.0752558708190918, -1.8977223835387171,
                         1.5743772983551025, 0.7854096293449402]

        # current_pos = ur16.getj()
        # if current_pos != fluidize_pose:
        #     print("Move to fluidize pose")
        #     data_logger.robot.movej(fluidize_pose, vel=50 / 1000, acc=0.5, wait=True)

        # data_logger.robot.movej(prepare_pose,vel=60/1000,acc=0.5,wait = True)

        # Start penetration
        # move_ur(ur16, moving_vector_down*80/1000, 3 / 1000, 1, wait=True)

        # data_logger.force_controlled_intrusion(intrusion_threshold=1.8)
        # print("Hit the ground. Start penetration 35mm")
        # Move 35mm down
        # move_ur(ur16, moving_vector_down * 35/1000, 0.5/1000, 1, wait=True)

        # rotate_around z
        print("Start rotating")
        rotate_around_h(ur16,(0,0,(179.99)*pi/180))
        time.sleep(3)
        rotate_around_h(ur16, (0, 0, (-179.99)*pi/180))
        time.sleep(3)

        data_logger.robot.movej(fluidize_pose, vel=50 / 1000, acc=0.5, wait=True)

        if ati_ip is not None:
            data_logger.ati_sensor.stop_streaming()
        data_logger.stop_logging()







        robot_angle = numpy.array(data_logger.robot_data)[:,-2]
        z_torque = numpy.array(data_logger.load_cell_data)[:,-1]
        Fx = numpy.array(data_logger.load_cell_data)[:,5]
        Fy = numpy.array(data_logger.load_cell_data)[:,6]
        robot_timestamp = numpy.array(data_logger.robot_data)[:,0]
        loadcell_timestamp = numpy.array(data_logger.load_cell_data)[:,0]

        robot_angle_interp = numpy.interp(loadcell_timestamp, robot_timestamp ,robot_angle)

        plt.figure(1)
        plt.plot(robot_timestamp,robot_angle)
        plt.plot(loadcell_timestamp,Fx, label="Fx")
        plt.plot(loadcell_timestamp, Fy, label="Fy")
        plt.legend()

        plt.figure(2)
        plt.plot(robot_angle_interp, Fx, label="Fx")
        plt.plot(robot_angle_interp, Fy, label="Fy")
        plt.legend()


        plt.show(block=True)



        # data_logger.robot.movej(fluidize_pose, vel=20 / 1000, acc=0.5, wait=True)

    except Exception as e:
        print(f"An error occurred: {e}")