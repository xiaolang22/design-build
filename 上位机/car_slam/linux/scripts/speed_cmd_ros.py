#!/usr/bin/env python3
"""
容器内速度控制程序
连接到主机的TCP服务，发送速度控制指令
支持ROS /cmd_vel 话题订阅
"""

import sys
import struct
import threading
import time
import socket
import rospy
from geometry_msgs.msg import Twist


class ContainerSpeedControl:
    def __init__(self, tcp_host='host.docker.internal', tcp_port=12345):
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port
        self.socket = None
        self.sending_active = False
        self.sending_thread = None

        # 当前速度值
        self.current_angular_speed = 0.0  # 度/秒
        self.current_linear_speed = 0.0  # 米/秒

        # 发送频率控制
        self.send_interval = 0.05  # 50ms, 20Hz
        self.packet_count = 0

        # ROS初始化
        rospy.init_node('container_speed_control', anonymous=True)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        print("容器速度控制节点初始化完成")
        print(f"目标TCP服务器: {tcp_host}:{tcp_port}")
        print("等待 /cmd_vel 话题消息...")

    def connect_tcp(self):
        """连接到TCP服务器"""
        try:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.tcp_host, self.tcp_port))
            self.socket.settimeout(1.0)  # 设置接收超时

            print(f"成功连接到TCP服务器: {self.tcp_host}:{self.tcp_port}")
            return True

        except Exception as e:
            print(f"TCP连接失败: {e}")
            return False

    def pack_speed_command(self, angular_speed, linear_speed):
        """
        打包速度控制指令
        格式: 帧头(0xFF22) + 角速度(float) + 线速度(float) + 帧尾(0x22EE)
        字节序: 帧头帧尾大端，数据小端
        """
        # 帧头: 0xFF22 (大端序)
        header = struct.pack('>H', 0xFF22)

        # 角速度: float (小端序，单位: °/s)
        angular = struct.pack('<f', angular_speed)

        # 线速度: float (小端序，单位: m/s)
        linear = struct.pack('<f', linear_speed)

        # 帧尾: 0x22EE (大端序)
        tail = struct.pack('>H', 0x22EE)

        data = header + angular + linear + tail

        # 调试信息
        if self.packet_count % 50 == 0:  # 每50个包打印一次
            hex_str = ' '.join([f'{b:02X}' for b in data])
            print(f"发送速度指令: 角速度={angular_speed:.1f}°/s, 线速度={linear_speed:.2f}m/s")
            print(f"数据包: {hex_str}")

        return data

    def send_speed_command(self, angular_speed, linear_speed):
        """发送速度控制指令"""
        if not self.socket:
            if not self.connect_tcp():
                return False

        try:
            data = self.pack_speed_command(angular_speed, linear_speed)
            self.socket.send(data)
            self.packet_count += 1
            return True

        except Exception as e:
            print(f"发送速度指令失败: {e}")
            # 尝试重新连接
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
            return False

    def cmd_vel_callback(self, msg):
        """
        ROS /cmd_vel 话题回调函数
        将ROS标准速度消息转换为控制指令
        """
        # ROS标准: 线速度 m/s, 角速度 rad/s
        linear_speed = msg.linear.x        #可能修改：反向控制
        angular_speed_rad = msg.angular.z  #可能修改：反向控制

        # 转换为控制指令需要的单位: 角速度从 rad/s 转换为 °/s
        angular_speed_deg = angular_speed_rad * 180.0 / 3.14159265359

        # 限制速度范围
        linear_speed = max(-1.0, min(1.0, linear_speed))
        angular_speed_deg = max(-180.0, min(180.0, angular_speed_deg))

        # 更新当前速度
        self.current_linear_speed = linear_speed
        self.current_angular_speed = angular_speed_deg

        # 发送速度指令
        self.send_speed_command(angular_speed_deg, linear_speed)

    def start_continuous_sending(self):
        """开始连续发送当前速度指令"""
        if self.sending_active:
            return

        self.sending_active = True
        self.sending_thread = threading.Thread(target=self._sending_worker)
        self.sending_thread.daemon = True
        self.sending_thread.start()
        print("开始连续发送速度指令")

    def stop_continuous_sending(self):
        """停止连续发送"""
        self.sending_active = False
        if self.sending_thread:
            self.sending_thread.join(timeout=1.0)
        print("停止连续发送速度指令")

    def _sending_worker(self):
        """连续发送工作线程"""
        last_status_time = time.time()

        while self.sending_active and not rospy.is_shutdown():
            success = self.send_speed_command(
                self.current_angular_speed,
                self.current_linear_speed
            )

            # 定期打印状态
            current_time = time.time()
            if current_time - last_status_time >= 5.0:
                if success:
                    print(f"连续发送状态: 角速度={self.current_angular_speed:.1f}°/s, "
                          f"线速度={self.current_linear_speed:.2f}m/s, "
                          f"总包数={self.packet_count}")
                else:
                    print("连续发送状态: 发送失败，尝试重新连接...")
                last_status_time = current_time

            time.sleep(self.send_interval)

    def set_speed(self, angular_speed_deg, linear_speed):
        """直接设置速度并发送"""
        self.current_angular_speed = angular_speed_deg
        self.current_linear_speed = linear_speed
        return self.send_speed_command(angular_speed_deg, linear_speed)

    def stop_motors(self):
        """停止电机"""
        print("发送停止指令")
        return self.set_speed(0.0, 0.0)

    def run(self):
        """主运行循环"""
        print("启动容器速度控制节点")

        # 初始连接
        if not self.connect_tcp():
            print("初始连接失败，将在收到速度指令时重试")

        # 开始连续发送
        self.start_continuous_sending()

        try:
            # ROS主循环
            rate = rospy.Rate(10)  # 10Hz
            while not rospy.is_shutdown():
                rate.sleep()

        except KeyboardInterrupt:
            print("用户中断程序")
        finally:
            self.stop_continuous_sending()
            self.stop_motors()
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
            print("容器速度控制节点已停止")


def main():
    # 从参数获取TCP连接信息，或使用默认值
    tcp_host = rospy.get_param('~tcp_host', 'host.docker.internal')
    tcp_port = rospy.get_param('~tcp_port', 12346)

    controller = ContainerSpeedControl(tcp_host, tcp_port)
    controller.run()


if __name__ == "__main__":
    main()