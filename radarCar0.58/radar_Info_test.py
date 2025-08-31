from robot_utils import Robot
from radar_interface import create_radar_interface
import time
import numpy as np
if __name__ == "__main__":
    # 创建机器人对象，假设初始位置为(0,0)，朝向0
    robot = Robot(5, 5, np.deg2rad(0))
    # 创建雷达接口
    radar = create_radar_interface(robot, 'maze_edges.json')
    # 测试多次调用并写入文件
    for i in range(5):
        distances = radar.get_current_radar_data(log_file='radar_info.dat')
        print(f"第{i+1}次调用，前5个距离值：{distances[:5]}")
        time.sleep(1)  # 间隔1秒，便于观察时间戳变化