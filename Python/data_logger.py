import urx
import threading
import time
import numpy as np
import csv
import os
import datetime
import ctypes  # For accessing C functions
import numpy

# Import your existing files
from read_ati_class_rdt import atiSensor  # Assuming this is your ATI class file
from control_ur_robot import move_ur,rotate_around_h


class DataLogger:
    def __init__(self, robot_ip, ati_sensor):
        self.robot = urx.Robot(robot_ip, use_rt=True, urFirm=5.9)
        self.ati_sensor = ati_sensor
        self.stop_event = threading.Event()

        self.robot_data = []
        self.load_cell_data = []

        time.sleep(5)
        self.initial_pose = self.robot.get_pos()

        self.index = 0  # Initialize data index

        threading.Thread(target=self.log_robot_data, daemon=True).start()
        threading.Thread(target=self.log_load_cell_data, daemon=True).start()



    def log_robot_data(self):
        while True:
            timestamp = time.time()
            pos_data = self.robot.get_pos() - self.initial_pose
            xyz = self.robot.get_pos()
            rx, ry, rz = self.robot.get_orientation().to_euler('xyz')
            isRunning_flag = int(self.robot.is_program_running())

            self.robot_data.append((timestamp, self.index, *xyz, rx, ry, rz,isRunning_flag))  # All data together
            self.index += 1
            time.sleep(0.005)

    def log_load_cell_data(self):
        while True:
            timestamp = time.time()
            data = self.ati_sensor.collect_sample()
            self.load_cell_data.append((timestamp, self.index, *data))  # All data together
            self.index += 1
            # time.sleep(0.00001)  # Adjusted for higher sampling rate
    def save_data(self):
        data_folder = "data"
        os.makedirs(data_folder, exist_ok=True)

        # Generate filename with current date and time
        now = datetime.datetime.now()
        timestamp_str = now.strftime("%Y_%m_%d_%H_%M")
        robot_filename = os.path.join(data_folder, f"{timestamp_str}_UR.csv")
        load_cell_filename = os.path.join(data_folder, f"{timestamp_str}_FT.csv")

        with open(robot_filename, 'w', newline='') as robot_file, open(load_cell_filename, 'w', newline='') as load_cell_file:
            robot_writer = csv.writer(robot_file)
            load_cell_writer = csv.writer(load_cell_file)

            robot_writer.writerow(["Timestamp", "Index","X", "Y", "Z", "Rx", "Ry", "Rz", "isRunning"])
            load_cell_writer.writerow(["Timestamp", "Index", "RDTSequence", "FTSerialNumber", "Status", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"])

            robot_writer.writerows(self.robot_data)
            load_cell_writer.writerows(self.load_cell_data)

        print(f"Data saved to {robot_filename} and {load_cell_filename}")

    def stop_logging(self):
        self.stop_event.set()  # Signal threads to stop gracefully

        # Wait a short period to allow graceful termination
        time.sleep(1)

        # Forcefully terminate threads if they haven't stopped
        for thread in threading.enumerate():
            if thread.name != "MainThread":
                thread_id = thread.ident
                res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, ctypes.py_object(SystemExit))
                if res > 1:
                    ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
                    print('Exception raise failure')


robot_ip = "192.168.0.110"
ur = urx.Robot(robot_ip, use_rt=False, urFirm=5.9)
time.sleep(5)
ati_sensor = atiSensor(tcp_ip="192.168.0.121")
data_logger = DataLogger(robot_ip, ati_sensor)

# Let the data logger run for some time...
time.sleep(3)

moving_vector_left = numpy.array((-1, 0, 0))
moving_vector_right = numpy.array((1, 0, 0))
moving_vector_forward = numpy.array((0, 1, 0))
moving_vector_backward = numpy.array((0, -1, 0))
moving_vector_up = numpy.array((0, 0, 1))
moving_vector_down = numpy.array((0, 0, -1))

## Do something over here for your exp
move_ur(ur,moving_vector_up*0.03,0.01,1,wait=True)

time.sleep(3)



data_logger.save_data()
ati_sensor.stop_streaming()
data_logger.stop_logging()  # Tell the threads to stop