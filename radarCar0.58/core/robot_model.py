import numpy as np

class Robot:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        """
        初始化机器人状态
        :param x: 初始x坐标（米）
        :param y: 初始y坐标（米）
        :param theta: 初始朝向（弧度）
        """
        self.xy = [x, y]
        self.theta = theta

    def step_move(self, direction, step):
        """
        步进模式下更新位姿
        :param direction: 0:前 1:右 2:后 3:左
        :param step: 步进距离（米）
        """
        if direction == 0:   # 前
            self.xy[1] += step
        elif direction == 1: # 右
            self.xy[0] += step
        elif direction == 2: # 后
            self.xy[1] -= step
        elif direction == 3: # 左
            self.xy[0] -= step
        # 步进模式不考虑朝向变化

    def velocity_move(self, vx, vy, omega, dt):
        """
        速度控制模式下更新位姿
        :param vx: x方向速度（米/秒）
        :param vy: y方向速度（米/秒）
        :param omega: 角速度（弧度/秒）
        :param dt: 时间步长（秒）
        """
        # 质点模型，直接积分
        self.xy[0] += vx * dt
        self.xy[1] += vy * dt
        self.theta += omega * dt
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi  # 归一化到[-pi, pi]

if __name__ == "__main__":
    # 最小自测
    robot = Robot()
    robot.step_move(0, 0.1)
    print(f"步进后位置: {robot.xy}, 朝向: {robot.theta}")
    robot.velocity_move(0.1, 0.0, 0.1, 1.0)
    print(f"速度控制后位置: {robot.xy}, 朝向: {robot.theta}") 