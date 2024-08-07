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
        self.combined_data = None
        self.use_robot = robot_ip is not None
        self.use_ati = ati_ip is not None
        self.UR_header = ["Timestamp", "Index","X", "Y", "Z", "Rx", "Ry", "Rz", "isRunning"]
        self.FT_header = ["Timestamp", "Index", "RDTSequence", "FTSerialNumber", "Status", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
        if not self.use_robot and not self.use_ati:
            raise ValueError("At least one data source (robot or ATI sensor) must be specified.")

        # Initialize robot connection (if needed)
        if self.use_robot:
            try:
                self.robot = urx.Robot(robot_ip, use_rt=True, urFirm=5.9)
                self.robot_data = []
                time.sleep(3)
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

    def start_recording(self):
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
            time.sleep(0.00000001)

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
                robot_writer.writerow(self.UR_header)
                robot_writer.writerows(self.robot_data)
                print(f"Data saved to {robot_filename} ")

        if self.use_ati:
            load_cell_filename = os.path.join(data_folder, f"{timestamp_str}_FT.csv")
            with open(load_cell_filename, 'w', newline='') as load_cell_file:
                load_cell_writer = csv.writer(load_cell_file)
                load_cell_writer.writerow(self.FT_header)
                load_cell_writer.writerows(self.load_cell_data)
                print(f"Data saved to {load_cell_filename} ")

        if self.use_robot and self.use_ati:
            robot_data = numpy.array(self.robot_data)
            load_cell_data = numpy.array(self.load_cell_data)

            ur_last_timestamp = robot_data[-1, 0]
            ft_last_timestamp = load_cell_data[-1, 0]
            if ur_last_timestamp < ft_last_timestamp:
                end_timestamp = ur_last_timestamp  # Use UR timestamp to limit FT data
                print("Warning: UR data ended before FT data. Combined data will be truncated.")
            else:
                end_timestamp = ft_last_timestamp  # Use FT timestamp to limit UR data
                print("Warning: FT data ended before UR data. Combined data will be truncated.")
            robot_data = robot_data[robot_data[:, 0] <= end_timestamp]
            load_cell_data = load_cell_data[load_cell_data[:, 0] <= end_timestamp]


            ati_sampling_rate = len(load_cell_data) / abs(load_cell_data[-1,0]-load_cell_data[0,0])
            flattened_timestamps = numpy.arange(len(load_cell_data)) / ati_sampling_rate
            # Interpolate UR data to match flattened timestamps (assuming 100 Hz)
            ur_timestamps = robot_data[:, 0]
            ur_values = robot_data[:, 2:]  # X, Y, Z, Rx, Ry, Rz, isRunning
            if not (ur_timestamps[0] <= flattened_timestamps[0] and ur_timestamps[-1] >= flattened_timestamps[-1]):
                print("Warning: UR timestamps fall outside the range of FT timestamps. Interpolation may be inaccurate.")
            interpolated_ur_values = numpy.zeros((len(flattened_timestamps), ur_values.shape[1]))
            for i in range(ur_values.shape[1]):  # Interpolate each column
                interpolated_ur_values[:, i] = numpy.interp(flattened_timestamps, ur_timestamps, ur_values[:, i])

            # Combine and save data
            # self.combined_data = numpy.column_stack((flattened_timestamps, interpolated_ur_values, load_cell_data[:, 2:]))
            combined_data = numpy.column_stack((flattened_timestamps, interpolated_ur_values, load_cell_data[:, 2:]))
            combined_filename = os.path.join(data_folder, f"{timestamp_str}_COMBO.csv")
            with open(combined_filename, 'w', newline='') as combined_file:
                combined_writer = csv.writer(combined_file)
                combined_writer.writerow(["Timestamp"] + self.UR_header[2:] + self.FT_header[2:])
                combined_writer.writerows(self.combined_data)
            print(f"Combined data saved to {combined_filename}")

            fig, (ax_x, ax_y) = plt.subplots(1, 2)
            ax_x.plot(self.combined_data[:, 1] * 1e3, self.combined_data[:, 11])
            ax_x.plot(self.combined_data[:, 1] * 1e3, self.combined_data[:, 12])
            ax_x.plot(self.combined_data[:, 1] * 1e3, self.combined_data[:, 13])
            ax_x.set_xlabel("X - Distance (mm)")
            ax_x.set_ylabel("Force (N)")
            ax_y.plot(self.combined_data[:, 2] * 1e3, self.combined_data[:, 11])
            ax_y.plot(self.combined_data[:, 2] * 1e3, self.combined_data[:, 12])
            ax_y.plot(self.combined_data[:, 2] * 1e3, self.combined_data[:, 13])
            ax_y.set_xlabel("Y - Distance (mm)")
            ax_y.set_ylabel("Force (N)")
            plt.show(block=True)


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
        if ati_ip is not None:
            data_logger.ati_sensor.stop_streaming()


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

        for jj in range(3):
            print(f"Dragging round #{jj+1}")
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