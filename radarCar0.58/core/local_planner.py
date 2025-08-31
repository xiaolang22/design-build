import numpy as np

# 步进模式控制器
class VehicleController:
    def __init__(self, step_max=0.2):
        """
        初始化步进控制器
        :param step_max: 单步最大距离（米）
        """
        self.step_max = step_max

    def move_vehicle(self, distance, direction):
        """
        执行步进移动，预测是否会撞墙（stub）
        :param distance: 步进距离（米）
        :param direction: 方向 0:前 1:右 2:后 3:左
        :return: bool 是否预测碰撞
        """
        # TODO: 调用雷达接口预测碰撞
        return False  # stub: 默认不碰撞


def planner2cmd(robot, nxt_xy, step_max=0.2):
    """
    计算从当前位姿到下一个目标点的步进指令
    :param robot: 机器人对象，需有xy属性
    :param nxt_xy: 下一个目标点(x, y)
    :param step_max: 单步最大距离
    :return: (direction, step)
    """
    dx = nxt_xy[0] - robot.xy[0]
    dy = nxt_xy[1] - robot.xy[1]
    if abs(dx) > abs(dy):
        direction = 1 if dx > 0 else 3  # 右/左
        step = min(abs(dx), step_max)
    else:
        direction = 0 if dy > 0 else 2  # 前/后
        step = min(abs(dy), step_max)
    return direction, step

# DWA模式控制器（stub）
class DWAController:
    def __init__(self):
        pass
    def plan(self, cmd_prev, goal_xy, occ_grid, robot):
        """
        DWA规划，输出速度指令（stub）
        :return: (vx, vy, omega)
        """
        # TODO: 实现DWA算法
        return 0.1, 0.0, 0.0  # stub: 恒定前进
    def update_robot(self, robot, dt):
        """
        积分更新机器人位姿（stub）
        """
        # TODO: 实现位姿积分
        pass

def local_plan(mode, *args, **kwargs):
    """
    局部规划统一接口
    :param mode: "step" 或 "dwa"
    :return: 步进指令或速度指令
    """
    if mode == "step":
        return planner2cmd(*args, **kwargs)
    elif mode == "dwa":
        dwa = kwargs.get('dwa_controller', DWAController())
        return dwa.plan(*args)
    else:
        raise ValueError(f"未知局部规划模式: {mode}")

if __name__ == "__main__":
    # 最小自测
    class RobotStub:
        def __init__(self, x, y):
            self.xy = [x, y]
    robot = RobotStub(0.0, 0.0)
    nxt_xy = (0.3, 0.0)
    dir_, step = planner2cmd(robot, nxt_xy)
    print(f"步进模式: direction={dir_}, step={step}")
    dwa = DWAController()
    vx, vy, w = dwa.plan(None, (1.0, 0.0), None, robot)
    print(f"DWA模式: vx={vx}, vy={vy}, w={w}") 