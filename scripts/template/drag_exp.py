# main_experiment.py
from UR_controller import UR_Controller
from math import pi
import numpy as np

if __name__ == "__main__":
    # 1. Setup
    ur5_ip = "192.168.0.110"
    ati_ip = "192.168.0.121"
    prepare_pose = [-2.02, -1.76, -1.61, -1.33, 1.57, -1.18]
    robot_depth = 0.07
    angle_of_attack = 90
    distance = 0.15

    # 2. Initialize
    exp = UR_Controller(ur5_ip, ati_ip)
    exp.connect_ur5()
    exp.connect_ati_sensor()
    exp.calibrate_ati_sensor()

    # 3. Move to start pose
    exp.ur5.movej(prepare_pose, vel=0.05, acc=1, wait=True)
    exp.ur5.set_digital_out(4, 1)
    time.sleep(5)
    exp.move_ur5(np.array([0, 0, -robot_depth]), v=0.01, a=0.1, wait=True)
    exp.rotate_tool(angle_of_attack * pi / 180, axis='rz')
    exp.ur5.set_digital_out(4, 0)
    time.sleep(15)

    # 4. Collect data
    exp.calibrate_ati_sensor()
    time_a = time.time()
    data = []
    data = exp.collect_one_line(time_a, data)
    data = exp.tictoc(data, time_a, pause=3)

    exp.set_tool_io(True)
    data = exp.tictoc(data, time_a, pause=3)

    mv_forward = np.array([0, 1, 0]) * distance
    mv_backward = np.array([0, -1, 0]) * distance

    for _ in range(5):
        data = exp.move_with_data(data, mv_backward, time_a)
        data = exp.tictoc(data, time_a, pause=1)
        data = exp.move_with_data(data, mv_forward, time_a)
        data = exp.tictoc(data, time_a, pause=1)

    exp.set_tool_io(False)
    exp.move_ur5(np.array([0, 0, robot_depth]), v=0.01, a=0.1, wait=True)
    exp.ur5.movej(prepare_pose, vel=0.05, acc=1, wait=True)

    # 5. Save and plot
    exp.save_data(data, prefix="body_qs", aoa=angle_of_attack)
    exp.plot_force(data)
    exp.plot_all(data)
