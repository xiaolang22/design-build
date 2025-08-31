from vehicle_control import create_vehicle_controller
from radar_interface import create_radar_interface
import numpy as np
import matplotlib.pyplot as plt

# 全局车辆控制器实例
vehicle_controller = create_vehicle_controller(15, 5, 0)
radar_interface = create_radar_interface(vehicle_controller.robot, 'maze_edges.txt')

radar = radar_interface.get_current_radar_data()
fname = f"radar_data_test.txt"
with open(fname, 'w', encoding='utf-8') as f:
    for v in radar:
        f.write(f"{v}\n")

# 绘制雷达方向
fig, ax = plt.subplots()

robot_x, robot_y, robot_theta = vehicle_controller.robot.x, vehicle_controller.robot.y, vehicle_controller.robot.theta
arrow_len = 2.0
ax.arrow(robot_x, robot_y, 
         arrow_len * np.cos(robot_theta), 
         arrow_len * np.sin(robot_theta), 
         head_width=0.5, head_length=0.7, fc='r', ec='r')

plt.show()