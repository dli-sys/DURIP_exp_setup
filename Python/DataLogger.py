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
import matplotlib.pyplot as plt
from read_ati_class_rdt import atiSensor  # Assuming this is your ATI class file
import queue

class DataLogger:
    def __init__(self, robot_ip=None, ati_ip=None):
        self.data_queue = queue.Queue()
        self.robot_data = []
        self.load_cell_data = []
        self.use_robot = robot_ip is not None
        self.use_ati = ati_ip is not None
        self.UR_header = ["Timestamp", "Index", "X", "Y", "Z", "Rx", "Ry", "Rz", "isRunning"]
        self.FT_header = ["Timestamp", "Index", "RDTSequence", "FTSerialNumber", "Status", "Fx", "Fy", "Fz", "Tx", "Ty",
                          "Tz"]

        # Initialize robot connection
        if self.use_robot:
            self.robot = urx.Robot(robot_ip, use_rt=True,urFirm=5.9)

        # Initialize ATI sensor connection
        if self.use_ati:
            self.ati_sensor = atiSensor(tcp_ip=ati_ip)

        # Shared timestamp origin
        self.initial_timestamp = time.perf_counter()
        self.stop_event = threading.Event()

    def flush(self):
        """Flush all buffers and reset indices."""
        print("Flushing data...")
        self.robot_data.clear()
        self.load_cell_data.clear()
        with self.data_queue.mutex:
            self.data_queue.queue.clear()
        print("Data flushed.")

    def log_robot_data(self):
        while not self.stop_event.is_set():
            try:
                master_timestamp = time.perf_counter()
                timestamp = master_timestamp - self.initial_timestamp
                xyz = self.robot.get_pos()
                rx, ry, rz = self.robot.get_orientation().to_euler('xyz')
                is_running_flag = int(self.robot.is_program_running())

                # Add to queue
                self.data_queue.put(("robot", timestamp, *xyz, rx, ry, rz, is_running_flag))
            except Exception as e:
                print(f"Robot data logging error: {e}")
            finally:
                pass
                # time.sleep(0.000001)  # Adjust for optimal performance

    def log_load_cell_data(self):
        while not self.stop_event.is_set():
            try:
                master_timestamp = time.perf_counter()
                timestamp = master_timestamp - self.initial_timestamp
                data = self.ati_sensor.collect_sample()

                # Add to queue
                self.data_queue.put(("ati", timestamp, *data))
            except Exception as e:
                print(f"Load cell logging error: {e}")
            finally:
                pass
                # time.sleep(0.00000001)  # Adjust for optimal performance

    def process_data(self):
        while not self.stop_event.is_set():
            try:
                # Batch process multiple items to reduce queue contention
                while not self.data_queue.empty():
                    data_type, timestamp, *values = self.data_queue.get()
                    if data_type == "robot":
                        self.robot_data.append((timestamp, *values))
                    elif data_type == "ati":
                        self.load_cell_data.append((timestamp, *values))
            except Exception as e:
                print(f"Data processing error: {e}")

    def start_logging(self):
        threading.Thread(target=self.process_data, daemon=True).start()
        if self.use_robot:
            threading.Thread(target=self.log_robot_data, daemon=True).start()
        if self.use_ati:
            threading.Thread(target=self.log_load_cell_data, daemon=True).start()

    def stop_logging(self):
        self.stop_event.set()
        # self.flush()
        if self.use_robot:
            self.robot.close()
        if self.use_ati:
            self.ati_sensor.stop_streaming()



    def move_ur(self, moving_vector, v, a, wait=False):

        current_pose = logger.robot.get_pose()
        # moving_vector = moving_vector_right*10/1000
        # v = 0.05
        # a = 0.1
        # wait = True
        # logger.robot.movel(current_pose, moving_vector, v, a, wait)

        current_pose = self.robot.get_pose()
        current_pose.pos[:] += moving_vector
        self.robot.movel(current_pose, vel=v, acc=a, wait=wait)

    def rotate_around_h(self, angle_r):
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
        pose = self.robot.get_pose()
        Tct = m3d.Transform()
        Tct.pos = m3d.Vector(0, 0, 0)
        Tct.orient = m3d.Orientation.new_euler(angle_r, encoding='XYZ')
        new_pos = pose * Tct
        self.robot.movel(new_pos, vel=0.03, acc=1, wait=True, threshold=5)

    def force_controlled_intrusion(self,step = 1/1000,intrusion_threshold=2):
        print("Start force controlled intrusion")
        moving_vector_down = numpy.array((0, 0, -1*step))
        calib_data_z = 0
        while calib_data_z < intrusion_threshold:
            time.sleep(0.1)
            desire_pose = self.robot.get_pose()
            desire_pose.pos[:]+=moving_vector_down
            self.robot.movel(desire_pose, acc=1, vel=1 / 1000, wait=True)
            # calib_data_list = 0
            # for j in range(7000):
            #     calib_data_list += self.load_cell_data[-1][7]
            print(self.load_cell_data[-1])
            # calib_data_z= abs(calib_data_list/7000)
            calib_data_z = abs(self.load_cell_data[-1][7])
            time.sleep(0.1)
            print(f"Current z reading: {calib_data_z}")

    def robot_pose_calibration(self):
        pass
    def save_data(self,append_exp_name=None):
        print("here e save data")
        try:
            self.stop_logging()
            timeout_duration = 5  # Timeout in seconds
            start_time = time.time()
            while not self.data_queue.empty() and time.time() - start_time < timeout_duration:
                time.sleep(0.1)  # Check every 0.1 seconds

            # Check if the queue is still not empty after the timeout
            if not self.data_queue.empty():
                print("Warning: Queue not empty after timeout. Some data might not be processed.")

            robot_data = self.robot_data
            load_cell_data = self.load_cell_data
            # load_cell_data = numpy.array(self.load_cell_data)

            data_folder = "data"
            os.makedirs(data_folder, exist_ok=True)

            # Generate filename with current date and time
            now = datetime.datetime.now()
            timestamp_str = now.strftime("%Y_%m_%d_%H_%M")
            if append_exp_name:
                timestamp_str += "_" + append_exp_name + "_"

            if self.use_robot:
                robot_filename = os.path.join(data_folder, f"{timestamp_str}_UR.csv")
                with open(robot_filename, 'w', newline='') as robot_file:
                    robot_writer = csv.writer(robot_file)
                    robot_writer.writerow(self.UR_header)
                    robot_writer.writerows(robot_data)
                    print(f"Data saved to {robot_filename} ")

            if self.use_ati:
                load_cell_filename = os.path.join(data_folder, f"{timestamp_str}_FT.csv")
                with open(load_cell_filename, 'w', newline='') as load_cell_file:
                    load_cell_writer = csv.writer(load_cell_file)
                    load_cell_writer.writerow(self.FT_header)
                    load_cell_writer.writerows(load_cell_data)
                    print(f"Data saved to {load_cell_filename} ")

            if self.use_robot and self.use_ati:
                robot_data = numpy.array(self.robot_data)
                load_cell_data = numpy.array(self.load_cell_data)

                ati_sampling_rate = len(load_cell_data) /abs(load_cell_data[-1,0]-load_cell_data[0,0])
                ati_flattened_timestamps = numpy.arange(len(load_cell_data)) / ati_sampling_rate
                load_cell_data[:,0] = ati_flattened_timestamps

                ur_last_timestamp = robot_data[-1, 0]
                ft_last_timestamp = load_cell_data[-1, 0]
                if ur_last_timestamp < ft_last_timestamp:
                    end_timestamp = ur_last_timestamp  # Use UR timestamp to limit FT data
                    print("Warning: UR data ended before FT data. ")
                else:
                    end_timestamp = ft_last_timestamp  # Use FT timestamp to limit UR data
                    print("Warning: FT data ended before UR data. ")
                robot_data = robot_data[robot_data[:, 0] <= end_timestamp]
                load_cell_data = load_cell_data[load_cell_data[:, 0] <= end_timestamp]

                ati_sampling_rate = len(load_cell_data) /abs(load_cell_data[-1,0]-load_cell_data[0,0])
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
                self.combined_data = numpy.column_stack((flattened_timestamps, interpolated_ur_values, load_cell_data[:, 2:]))
                # combined_data = numpy.column_stack((flattened_timestamps, interpolated_ur_values, load_cell_data[:, 2:]))
                combined_filename = os.path.join(data_folder, f"{timestamp_str}_COMBO.csv")
                with open(combined_filename, 'w', newline='') as combined_file:
                    combined_writer = csv.writer(combined_file)
                    combined_writer.writerow(["Timestamp"] + self.UR_header[2:] + self.FT_header[2:])
                    combined_writer.writerows(self.combined_data)
                print(f"Combined data saved to {combined_filename}")

                fig, ([ax_x, ax_y,ax_z],[tb_ur,tb_ft,tb_angle]) = plt.subplots(2, 3)
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

                ax_z.plot(self.combined_data[:, 3] * 1e3, self.combined_data[:, 11])
                ax_z.plot(self.combined_data[:, 3] * 1e3, self.combined_data[:, 12])
                ax_z.plot(self.combined_data[:, 3] * 1e3, self.combined_data[:, 13])
                ax_z.set_xlabel("Z - Distance (mm)")
                ax_z.set_ylabel("Force (N)")

                tb_ur.plot(self.combined_data[:, 0], self.combined_data[:, 1] * 1e3)
                tb_ur.plot(self.combined_data[:, 0], self.combined_data[:, 2] * 1e3)
                tb_ur.plot(self.combined_data[:, 0], self.combined_data[:, 3] * 1e3)
                tb_ur.set_xlabel("Time (s)")
                tb_ur.set_ylabel("X - Distance (mm)")

                tb_ft.plot(self.combined_data[:, 0], self.combined_data[:, 11])
                tb_ft.plot(self.combined_data[:, 0], self.combined_data[:, 12])
                tb_ft.plot(self.combined_data[:, 0], self.combined_data[:, 13])
                tb_ft.set_xlabel("Time (s)")
                tb_ft.set_ylabel("Force (N)")

                tb_angle.plot(self.combined_data[:, 0], numpy.rad2deg(self.combined_data[:, 4]))
                tb_angle.plot(self.combined_data[:, 0], numpy.rad2deg(self.combined_data[:, 5]))
                tb_angle.plot(self.combined_data[:, 0], numpy.rad2deg(self.combined_data[:, 6]))
                tb_angle.set_xlabel("Time (s)")
                tb_angle.set_ylabel("Force (N)")

                plt.show(block=True)

        except Exception as e:
            print(f"An error occurred during plotting: {e}")


if __name__ == '__main__':

    ROBOT_IP = "192.168.0.110"  # Example robot IP
    ATI_IP = "192.168.0.121"    # Example ATI sensor IP

    moving_vector_left = numpy.array((-1,0,0))
    moving_vector_right = numpy.array((1,0,0))
    moving_vector_forward = numpy.array((0,1,0))
    moving_vector_backward = numpy.array((0,-1,0))
    moving_vector_up = numpy.array((0,0,1))
    moving_vector_down = numpy.array((0,0,-1))

    try:
        # Initialize DataLogger with both robot and ATI sensor
        logger = DataLogger(robot_ip=ROBOT_IP, ati_ip=ATI_IP)
        # time.sleep(10)

        print("Starting data logging...")
        logger.start_logging()


        logger.move_ur(moving_vector_down /1000 * 50, v=10 / 1000, a=1, wait=True)
        time.sleep(2)
        logger.move_ur(moving_vector_up/1000*50,v=10/1000,a=1, wait=True)

        print("Stopping data logging...")
        logger.stop_logging()

        print("Saving data for verification...")
        matplotlib.use('TkAgg')
        logger.save_data(append_exp_name="test_run")
        plt.show(block=True)
        input("Press Enter to exit after viewing the plot.")
        # Flush data (clears buffers and queues)
        # logger.flush()

        # Basic statistics for verification
        if logger.robot_data:
            print(f"Robot data collected: {len(logger.robot_data)} entries")
            print(f"Robot timestamp range: {np.min(logger.robot_data, axis=0)[0]} - {np.max(logger.robot_data, axis=0)[0]}")

        if logger.load_cell_data:
            print(f"Load cell data collected: {len(logger.load_cell_data)} entries")
            print(f"Load cell timestamp range: {np.min(logger.load_cell_data, axis=0)[0]} - {np.max(logger.load_cell_data, axis=0)[0]}")

    except Exception as e:
        print(f"An error occurred: {e}")
        logger.stop_logging()
        logger.save_data(append_exp_name="test_run")

    finally:
        # Ensure proper cleanup
        if logger.use_robot:
            logger.robot.close()
        if logger.use_ati:
            logger.ati_sensor.stop_streaming()