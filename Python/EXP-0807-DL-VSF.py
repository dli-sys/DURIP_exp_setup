# HKJF
import time
import numpy
from DataLogger import DataLogger


if __name__ == '__main__':
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
        print(f"Current robot joint angle: {ur16.getj()} ")
        print(f"Current robot joint angle: {numpy.rad2deg(ur16.getj())} ")

        prepare_pose = [-1.7903106848346155, -1.924272199670309, -1.8609415292739868, -0.9263626498034974, 1.5695431232452393, 0.5676261186599731]
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

        # start recording
        data_logger.flush()
        # start recording

        data_logger.start_recording()
        time.sleep(2)
        # data_logger.flush()

        # rotate_around z
        print("Start dragging")
        repeat_time = 1
        test_vel = 20/1000
        for jj in range(repeat_time):
            print(f"Dragging round #{jj+1}/{repeat_time}")
            data_logger.move_ur(moving_vector_left * 50 / 1000, test_vel, 1, wait=True)
            # time.sleep(1)
            # data_logger.move_ur(moving_vector_left * 100 / 1000, test_vel, 1, wait=True)
            # time.sleep(1)

        time.sleep(3)
        data_logger.robot.movej(exp_pose, vel=test_vel, acc=1, wait=True)
        time.sleep(2)
        data_logger.save_data(append_exp_name=exp_prefix)


    except KeyboardInterrupt:
        data_logger.save_data(append_exp_name=exp_prefix)
        data_logger.stop_logging()

    except Exception as e:
        data_logger.stop_logging()
        print(f"An error occurred: {e}")

