#!/usr/bin/env python3
import rospy
import os
import sys
import select
import termios
import tty
from datetime import datetime
from std_srvs.srv import Trigger, TriggerResponse


class KeyboardMapSaver:
    def __init__(self):
        rospy.init_node('keyboard_map_saver')

        # 获取保存路径参数
        self.save_dir = rospy.get_param('~save_dir', '~/cartographer_maps')
        self.save_dir = os.path.expanduser(self.save_dir)

        # 创建保存目录
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            rospy.loginfo("创建地图保存目录: %s", self.save_dir)

        # 保存设置
        self.save_pbstream = rospy.get_param('~save_pbstream', False)

        # 键盘设置
        self.settings = termios.tcgetattr(sys.stdin)
        self.key_timeout = 0.1

        rospy.loginfo("键盘地图保存节点已启动")
        rospy.loginfo("按 's' 键保存当前地图")
        rospy.loginfo("按 'q' 键退出")
        rospy.loginfo("地图将保存到: %s", self.save_dir)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], self.key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def save_map(self):
        """保存地图"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_name = f"map_{timestamp}"
        map_path = os.path.join(self.save_dir, map_name)

        try:
            # 保存为 ROS 地图格式 (.pgm + .yaml)
            command = f"rosrun map_server map_saver -f {map_path}"
            os.system(command)
            rospy.loginfo("✅ 地图已保存: %s.pgm", map_name)

            # 如果配置了，同时保存 Cartographer 完整状态
            if self.save_pbstream:
                # 调用 Cartographer 服务保存完整状态
                try:
                    rospy.wait_for_service('/finish_trajectory', timeout=2.0)
                    rospy.wait_for_service('/write_state', timeout=2.0)

                    finish_trajectory = rospy.ServiceProxy('/finish_trajectory', Trigger)
                    write_state = rospy.ServiceProxy('/write_state', Trigger)

                    finish_trajectory()
                    write_state(filename=f"{map_path}.pbstream")
                    rospy.loginfo("✅ Cartographer 状态已保存: %s.pbstream", map_name)

                except rospy.ServiceException as e:
                    rospy.logwarn("无法保存 Cartographer 状态: %s", str(e))

        except Exception as e:
            rospy.logerr("保存地图失败: %s", str(e))

    def run(self):
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                if key == 's':
                    self.save_map()
                elif key == 'q':
                    rospy.loginfo("退出键盘地图保存节点")
                    break
                elif key != '':
                    rospy.loginfo("按 's' 保存地图, 'q' 退出")

        except Exception as e:
            rospy.logerr("节点运行错误: %s", str(e))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == '__main__':
    try:
        node = KeyboardMapSaver()
        node.run()
    except rospy.ROSInterruptException:
        pass