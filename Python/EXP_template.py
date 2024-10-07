# HKJF
import time
import numpy
from numpy import pi

# Import your existing files
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
            user_input = input("Use current robot position as starting point? (Enter/N): ").strip().upper()
            if user_input == '':
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
        dump_timestamp = time.time()
        print(time.time()-dump_timestamp)
        data_logger.fluh()
        print(time.time() - dump_timestamp)
        # start recording
        data_logger.start_recording()
        print(time.time() - dump_timestamp)
        data_logger.flush()
        print(time.time() - dump_timestamp)
        time.sleep(5)
        print(time.time() - dump_timestamp)
        # data_logger.force_controlled_intrusion(intrusion_threshold=0.8)
        #time.sleep(2)


        # rotate_around z
        #print("Start dragging")
        # rotate_around_h(ur16,(0,0,(-179)*pi/180))
        repeat_time = 1
        test_vel = 15/1000
        for jj in range(repeat_time):
            data_logger.move_ur(ur16, moving_vector_right * 100 / 1000, 1 / 1000, 1, wait=True)
            time.sleep(1)
            data_logger.move_ur(ur16, moving_vector_left * 100 / 1000, 1 / 1000, 1, wait=True)
            time.sleep(1)
        time.sleep(2)
        data_logger.robot.movej(exp_pose, vel=test_vel, acc=0.5, wait=True)

        time.sleep(2)

        data_logger.save_data(append_exp_name=exp_prefix)
        data_logger.stop_logging()


    except KeyboardInterrupt:
        data_logger.save_data(append_exp_name=exp_prefix)
        data_logger.stop_logging()

    except Exception as e:
        data_logger.stop_logging()
        print(f"An error occurred: {e}")
