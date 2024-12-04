# HKJF
import time
import numpy
from numpy import pi
import matplotlib.pyplot as plt

# Import your existing files
from DataLogger import DataLogger


if __name__ == '__main__':


    ROBOT_IP    = "192.168.0.110"  # Example robot IP
    ATI_IP      = "192.168.0.121"    # Example ATI sensor IP

    moving_vector_left      = numpy.array((-1,0,0))
    moving_vector_right     = numpy.array((1,0,0))
    moving_vector_forward   = numpy.array((0,1,0))
    moving_vector_backward  = numpy.array((0,-1,0))
    moving_vector_up        = numpy.array((0,0,1))
    moving_vector_down      = numpy.array((0,0,-1))



    try:
        user_input = input("Add custom data prefix?").strip().upper()
        if user_input is not "":
            exp_prefix = user_input
        else:
            exp_prefix = None

        ati_ip = "192.168.0.121"
        robot_ip = "192.168.0.110"
        data_logger = DataLogger(ati_ip=ati_ip,robot_ip=robot_ip)

        # Robot moving loop
        # To access the robot, one can use
        ur16 = data_logger.robot
        # Define robot variables
        # tcp = ((0, 0, 0.30, 0, 0, 0))
        # payload_m = 0.1
        # payload_location = (0, 0, 0.15)
        # ur16.set_tcp(tcp)
        # ur16.set_payload(payload_m, payload_location)
        print(f"Current robot location: {ur16.get_pos()}")
        print(f"Current robot joint angle: {ur16.getj()} ")
        print(f"Current robot joint angle: {numpy.rad2deg(ur16.getj())} ")



        prepare_pose = []

        while True:
            user_input = input("Use current robot position as starting point? (Enter/N): ").strip().upper()
            if user_input == '':
                exp_pose = ur16.getj()  # Update if user says yes
                break
            elif user_input == 'N':
                exp_pose = prepare_pose
                data_logger.robot.movej(exp_pose, vel=10 / 1000, acc=0.5, wait=True)
                break
            else:
                print("Invalid input. Please enter 'enter' or N.")

        # move_ur(ur16, moving_vector_down*80/1000, 3 / 1000, 1, wait=True)

        ini_position = ur16.get_pos()[:]

        print("Starting data logging...")
        data_logger.start_logging()

        repeat_time = 1
        test_vel = 150/1000

        # ur16.set_tool_voltage(7.4)  # Here is just a function for vibration

        for jj in range(repeat_time):
            data_logger.move_ur(ur16, moving_vector_right * 100 / 1000, 1 / 1000, 1, wait=True)
            time.sleep(1)

            data_logger.move_ur(ur16, moving_vector_left * 100 / 1000, 1 / 1000, 1, wait=True)
            time.sleep(1)


        print("Stopping data logging...")
        data_logger.stop_logging()

        print("Saving data for verification...")
        data_logger.save_data(append_exp_name="test_run")
        plt.show(block=True)
        input("Press Enter to exit after viewing the plot.")

        data_logger.robot.movej(exp_pose, vel=test_vel, acc=0.5, wait=True)
        time.sleep(2)

        # Basic statistics for verification
        if data_logger.robot_data:
            print(f"Robot data collected: {len(data_logger.robot_data)} entries")
            print(
                f"Robot timestamp range: {numpy.min(data_logger.robot_data, axis=0)[0]} - {numpy.max(data_logger.robot_data, axis=0)[0]}")

        if data_logger.load_cell_data:
            print(f"Load cell data collected: {len(data_logger.load_cell_data)} entries")
            print(
                f"Load cell timestamp range: {numpy.min(data_logger.load_cell_data, axis=0)[0]} - {numpy.max(data_logger.load_cell_data, axis=0)[0]}")

    except Exception as e:
        print(f"An error occurred: {e}")
        data_logger.stop_logging()
        data_logger.save_data(append_exp_name="test_run")

    finally:
        # Ensure proper cleanup
        if data_logger.use_robot:
            data_logger.robot.close()
        if data_logger.use_ati:
            data_logger.ati_sensor.stop_streaming()
