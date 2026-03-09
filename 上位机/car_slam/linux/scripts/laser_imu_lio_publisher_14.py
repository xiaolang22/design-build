#!/usr/bin/env python3
"""""""""""""""""""""""""""""""""""""""""""""
    节点名：laser_imu_lio_publisher
    本代码功能：
    1、解析tcp数据帧
    2、发布/scan、/imu、/odom话题、odom到base_link的tf
    
    适配新的数据帧结构：572字节/帧
    增加了IMU欧拉角数据并转换为四元数,并将四元数添加到/imu话题, yaw角角度制转弧度制
    新增里程计tf发布和/odom话题发布：使用AB轮平均距离和yaw角计算位置
    
    删除odom到base_link的tf发送功能
"""""""""""""""""""""""""""""""""""""""""""""
import math
import socket
import struct
import numpy as np
import time
import rospy
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, Twist, Vector3
from std_msgs.msg import Header
import tf
from tf.transformations import quaternion_from_euler
from math import radians, sin, cos, atan2, degrees


class RPLidarC1ParserTCP:
    def __init__(self, host='host.docker.internal', port=12345):
        # TCP连接参数
        self.host = host
        self.port = port
        self.socket = None

        # 帧结构参数
        # 新的帧结构: header(2) + imu_raw_data[6](24) + imu_rpy_data[3](12) + wheelA_dis(8) + wheelB_dis(8) +
        #         speed_linear(8) + point_count(2) + angle_resolution(4) + distances[250](500) + checksum(2) + tail(2)
        # 总大小: 2 + 24 + 12 + 8 + 8 + 8 + 2 + 4 + 500 + 2 + 2 = 572字节
        self.frame_format = '>H6f3f3dHf250HHH'  # 大端序: header + 6float + 3float + 3double + ushort + float + 250ushort + ushort + ushort
        self.frame_size = struct.calcsize(self.frame_format)
        # 期望的帧头和帧尾
        self.expected_header = 0xFF77
        self.expected_tail = 0x77EE

        ## 雷达参数
        # 雷达规格参数（固定）
        self.num_points = 250
        self.angle_resolution = 1.44  # 度
        self.max_range = 12.0  # 米
        self.min_range = 0.05  # 米
        self.scan_frequency = 10  # Hz
        # 激光扫描消息模板 - 使用250个点
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = "laser_link"
        self.scan_msg.range_min = self.min_range
        self.scan_msg.range_max = self.max_range
        self.scan_msg.angle_min = 0.0
        self.scan_msg.angle_max = 2 * np.pi - np.radians(self.angle_resolution)  # 359.856度，避免360度重复
        self.scan_msg.angle_increment = np.radians(self.angle_resolution)
        self.scan_msg.scan_time = 1.0 / self.scan_frequency
        self.scan_msg.time_increment = self.scan_msg.scan_time / self.num_points
        self.scan_msg.ranges = [float('inf')] * self.num_points  # 使用250个点
        # 距离数据数组 - 现在直接使用250个点，不映射到360度
        self.distance_array = np.full(250, float('inf'), dtype=np.float32)

        ## IMU参数
        self.imu_raw_data = [0.0] * 6  # [ax, ay, az, gx, gy, gz]
        self.imu_rpy_data = [0.0] * 3  # [roll, pitch, yaw] - 新增的欧拉角数据
        self.imu_sequence = 0
        # IMU消息模板
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"
        # 设置IMU消息的协方差矩阵
        self.imu_msg.linear_acceleration_covariance = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        ]
        self.imu_msg.angular_velocity_covariance = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        ]
        self.imu_msg.orientation_covariance = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        ]

        # 编码器数据
        self.wheelA_distance = 0.0
        self.wheelB_distance = 0.0
        self.speed_linear = 0.0  # 新增线速度数据
        self.last_wheelA_distance = 0.0
        self.last_wheelB_distance = 0.0
        self.last_yaw = 0.0

        # 里程计参数
        self.odom_x = 0.0  # x位置（米）
        self.odom_y = 0.0  # y位置（米）
        self.odom_yaw = 0.0  # 偏航角（弧度）
        self.last_odom_update_time = time.time()

        # TF广播器
        # self.tf_broadcaster = tf.TransformBroadcaster()

        # ROS发布器
        rospy.init_node('laser_imu_lio_publisher', anonymous=True)
        self.laser_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)  # 新增odom话题发布

        # 统计信息
        self.bytes_received = 0
        self.frames_parsed = 0
        self.frames_failed = 0
        self.reconnect_count = 0
        self.last_data_time = 0
        self.last_status_time = 0
        self.last_publish_time = 0

        print("RPLIDAR C1 TCP解析器初始化完成，ROS节点已启动")
        print("支持雷达数据发布到 /scan 话题")
        print("支持IMU数据发布到 /imu 话题")
        # print("支持里程计tf发布：odom到base_link")
        print("支持里程计数据发布到 /odom 话题")
        print(f"数据帧大小: {self.frame_size} 字节")
        print(f"使用 {self.num_points} 个雷达数据点，角度分辨率: {self.angle_resolution}度")

    def connect_tcp(self):
        """连接TCP服务器"""
        try:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # 设置TCP选项
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)

            # 连接参数
            self.socket.settimeout(10.0)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(1.0)  # 接收数据超时1秒

            print(f"成功连接到TCP服务器: {self.host}:{self.port}")
            self.last_data_time = time.time()
            return True

        except Exception as e:
            print(f"TCP连接失败: {e}")
            return False

    def check_connection_health(self):
        """检查连接健康状态"""
        current_time = time.time()

        # 如果超过15秒没有收到数据，认为连接有问题
        if current_time - self.last_data_time > 15.0 and self.frames_parsed > 0:
            print("连接健康检查失败：长时间没有收到数据")
            return False

        return True

    def parse_frame(self, data):
        """解析激光雷达、IMU和编码器数据帧"""
        if len(data) != self.frame_size:
            print(f"数据长度不匹配: 期望 {self.frame_size}, 实际 {len(data)}")
            return None

        try:
            # 解包数据
            unpacked_data = struct.unpack(self.frame_format, data)

            # 提取各个字段 (根据新的帧结构)
            header = unpacked_data[0]
            imu_raw_data = unpacked_data[1:7]  # 6个IMU原始数据                          -加速度计g，陀螺仪°/s
            imu_rpy_data = unpacked_data[7:10]  # 3个IMU欧拉角数据 [roll, pitch, yaw]     -度
            wheelA_dis = unpacked_data[10]  # A轮距离 (double) - 单位：毫米
            wheelB_dis = unpacked_data[11]  # B轮距离 (double) - 单位：毫米
            speed_linear = unpacked_data[12]  # 线速度 (double) - 单位：米/秒
            point_count = unpacked_data[13]  # 点数
            angle_res = unpacked_data[14]  # 角度分辨率
            distances = unpacked_data[15:265]  # 250个距离数据
            checksum = unpacked_data[265]  # 校验和
            tail = unpacked_data[266]  # 帧尾

            # 验证帧头和帧尾
            if header != self.expected_header:
                print(f"帧头验证失败: 期望 0x{self.expected_header:04X}, 实际 0x{header:04X}")
                return None

            if tail != self.expected_tail:
                print(f"帧尾验证失败: 期望 0x{self.expected_tail:04X}, 实际 0x{tail:04X}")
                return None

            return {
                'imu_raw_data': list(imu_raw_data),  # [ax, ay, az, gx, gy, gz]
                'imu_rpy_data': list(imu_rpy_data),  # [roll, pitch, yaw] - 新增
                'wheelA_dis': wheelA_dis,  # A轮距离 - 毫米
                'wheelB_dis': wheelB_dis,  # B轮距离 - 毫米
                'speed_linear': speed_linear,  # 线速度 - 米/秒
                'point_count': point_count,  # 点数
                'angle_resolution': angle_res,  # 角度分辨率
                'distances': list(distances),  # 距离数据
                'checksum': checksum  # 校验和
            }

        except struct.error as e:
            print(f"解包数据时出错: {e}")
            return None
        except Exception as e:
            print(f"解析数据帧时发生未知错误: {e}")
            return None

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        将欧拉角转换为四元数
        假设pitch和roll角为0，只有偏航角yaw
        要求传进来的单位是弧度
        """
        # 由于假设pitch和roll为0，我们只需要处理yaw
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        w = cy * cr * cp + sy * sr * sp
        x = cy * sr * cp - sy * cr * sp
        y = cy * cr * sp + sy * sr * cp
        z = sy * cr * cp - cy * sr * sp

        return (x, y, z, w)

    def update_odometry(self, wheelA_dis, wheelB_dis, yaw_deg, speed_linear):
        """更新里程计位置和姿态，同时发布/odom话题和odom到base_link的tf"""
        current_time = time.time()
        dt = current_time - self.last_odom_update_time

        if dt <= 0:
            return

        # 计算平均轮子距离变化量（将毫米转换为米）
        avg_distance_m = (wheelA_dis + wheelB_dis) / 2.0 / 1000.0  # 转换为米
        last_avg_distance_m = (self.last_wheelA_distance + self.last_wheelB_distance) / 2.0 / 1000.0  # 转换为米
        d_distance = avg_distance_m - last_avg_distance_m

        # 将yaw从角度转换为弧度 (-180到+180度转换为-π到+π弧度)
        yaw_rad = radians(yaw_deg)

        # 计算位置变化量 - 假设在短时间内直线运动
        dx = d_distance * cos(yaw_rad)
        dy = d_distance * sin(yaw_rad)

        # 更新累计位置
        self.odom_x += dx   #m
        self.odom_y += dy   #m
        self.odom_yaw = yaw_rad #弧度

        # 保存当前值用于下一次计算
        self.last_wheelA_distance = wheelA_dis  #mm
        self.last_wheelB_distance = wheelB_dis  #mm
        self.last_yaw = yaw_deg                 #度
        self.speed_linear = speed_linear  # 保存线速度
        self.last_odom_update_time = current_time

        # 发布tf变换和odom话题
        # self.publish_odom_tf()
        self.publish_odom_msg()

    def publish_odom_tf(self):
        """发布odom到base_link的tf变换"""
        current_time = rospy.Time.now()

        # 创建四元数表示偏航角
        q = quaternion_from_euler(0, 0, self.odom_yaw)

        # 发布tf变换
        self.tf_broadcaster.sendTransform(
            (self.odom_x, self.odom_y, 0.0),  # 位置 (x, y, z)
            q,  # 四元数 (x, y, z, w)
            current_time,  # 时间戳
            "base_link",  # 子坐标系
            "odom"  # 父坐标系
        )

    def publish_odom_msg(self):
        """发布Odometry消息到/odom话题"""
        current_time = rospy.Time.now()

        # 创建Odometry消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # 设置位置
        odom_msg.pose.pose.position = Point(self.odom_x, self.odom_y, 0.0)

        # 设置方向（四元数）
        q = quaternion_from_euler(0, 0, self.odom_yaw)
        odom_msg.pose.pose.orientation = Quaternion(*q)

        # 设置速度
        odom_msg.twist.twist.linear.x = self.speed_linear #m/s
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        # 角速度从IMU获取（已经转换为弧度/秒）
        odom_msg.twist.twist.angular.x = self.imu_raw_data[3]
        odom_msg.twist.twist.angular.y = self.imu_raw_data[4]
        odom_msg.twist.twist.angular.z = self.imu_raw_data[5]

        # 发布消息
        self.odom_pub.publish(odom_msg)

    def update_distance_array(self, distances):
        """更新距离数组 - 直接使用250个点，不进行360度映射"""
        # 重置距离数组
        self.distance_array.fill(float('inf'))

        for i, distance_mm in enumerate(distances):
            if i >= self.num_points:  # 确保不超过点数限制
                break

            # 验证距离数据有效性
            if 50 <= distance_mm <= 12000:  # 5cm 到 12m 范围内
                distance_m = distance_mm / 1000.0
                self.distance_array[i] = distance_m
            else:
                self.distance_array[i] = float('inf')  # 使用inf表示无效数据

    def update_imu_data(self, imu_raw_data, imu_rpy_data):
        """更新IMU数据"""
        if len(imu_raw_data) == 6:
            # IMU原始数据单位检查：
            # imu_raw_data: [ax, ay, az, gx, gy, gz]
            # 根据注释，加速度单位是g，角速度单位是°/s
            # 对于ROS标准，需要转换为：
            # 加速度: m/s² (1g = 9.8 m/s²)
            # 角速度: rad/s
            self.imu_raw_data = [
                imu_raw_data[0] * 9.8,  # ax: g -> m/s²
                imu_raw_data[1] * 9.8,  # ay: g -> m/s²
                imu_raw_data[2] * 9.8,  # az: g -> m/s²
                0,                      # gx: °/s -> rad/s  假设为0
                0,                      # gy: °/s -> rad/s  假设为0
                radians(imu_raw_data[5])  # gz: °/s -> rad/s
            ]
        if len(imu_rpy_data) == 3:
            self.imu_rpy_data = imu_rpy_data
        return True

    def update_encoder_data(self, wheelA_dis, wheelB_dis, speed_linear):
        """更新编码器数据"""
        self.wheelA_distance = wheelA_dis
        self.wheelB_distance = wheelB_dis
        self.speed_linear = speed_linear
        return True

    def create_laserscan_msg(self):
        """创建ROS LaserScan消息 - 直接使用250个点"""
        current_time = rospy.Time.now()
        self.scan_msg.header.stamp = current_time

        # 直接使用距离数组中的数据
        for i in range(self.num_points):
            if self.distance_array[i] < float('inf'):
                self.scan_msg.ranges[i] = self.distance_array[i]
            else:
                self.scan_msg.ranges[i] = float('inf')

        return self.scan_msg

    def create_imu_msg(self):
        """创建ROS IMU消息 - 使用欧拉角转换为四元数"""
        current_time = rospy.Time.now()

        # 更新IMU消息头
        self.imu_msg.header.stamp = current_time
        self.imu_msg.header.seq = self.imu_sequence
        self.imu_sequence += 1

        # 设置线性加速度 (m/s^2) - 已经在update_imu_data中转换
        self.imu_msg.linear_acceleration.x = self.imu_raw_data[0]  # ax
        self.imu_msg.linear_acceleration.y = self.imu_raw_data[1]  # ay
        self.imu_msg.linear_acceleration.z = self.imu_raw_data[2]  # az

        # 设置角速度 (rad/s) - 已经在update_imu_data中转换
        self.imu_msg.angular_velocity.x = self.imu_raw_data[3]  # gx
        self.imu_msg.angular_velocity.y = self.imu_raw_data[4]  # gy
        self.imu_msg.angular_velocity.z = self.imu_raw_data[5]  # gz

        # 使用欧拉角转换为四元数
        roll, pitch, yaw = self.imu_rpy_data
        yaw_rad = radians(yaw)  # 使用弧度制
        # 假设pitch和roll角为0，只有偏航角yaw
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw_rad)

        self.imu_msg.orientation.x = qx
        self.imu_msg.orientation.y = qy
        self.imu_msg.orientation.z = qz
        self.imu_msg.orientation.w = qw

        return self.imu_msg

    def process_data_stream(self):
        """处理数据流 - 同时发布雷达、IMU和里程计数据"""
        buffer = b''
        search_state = "SEARCHING"

        # 发布频率控制 (10Hz)
        publish_interval = 0.1  # 100ms
        last_publish_time = 0

        while not rospy.is_shutdown():
            try:
                # 检查连接健康状态
                if not self.check_connection_health():
                    return False

                # 从TCP读取数据
                data = self.socket.recv(8192)

                if data:
                    self.bytes_received += len(data)
                    buffer += data
                    self.last_data_time = time.time()

                    # 数据解析逻辑
                    if search_state == "SEARCHING":
                        header_pos = buffer.find(b'\xFF\x77')
                        if header_pos >= 0:
                            if header_pos > 0:
                                # 丢弃帧头前的无效数据
                                buffer = buffer[header_pos:]
                            search_state = "SYNCED"
                            if self.frames_parsed == 0:
                                print("帧同步成功，开始解析数据帧")

                    if search_state == "SYNCED":
                        # 处理所有完整帧
                        while len(buffer) >= self.frame_size:
                            frame_data = buffer[:self.frame_size]
                            buffer = buffer[self.frame_size:]

                            parsed_data = self.parse_frame(frame_data)

                            if parsed_data:
                                self.frames_parsed += 1
                                self.last_data_time = time.time()

                                # 更新IMU数据（包括原始数据和欧拉角数据）
                                self.update_imu_data(
                                    parsed_data['imu_raw_data'],
                                    parsed_data['imu_rpy_data']
                                )

                                # 更新编码器数据
                                self.update_encoder_data(
                                    parsed_data['wheelA_dis'],
                                    parsed_data['wheelB_dis'],
                                    parsed_data['speed_linear']
                                )

                                # 更新距离数组 (直接使用250个点，不进行360度映射)
                                self.update_distance_array(parsed_data['distances'])

                                # 更新里程计
                                yaw_deg = parsed_data['imu_rpy_data'][2]
                                self.update_odometry(
                                    parsed_data['wheelA_dis'],
                                    parsed_data['wheelB_dis'],
                                    yaw_deg,
                                    parsed_data['speed_linear']
                                )

                                # 控制发布频率
                                current_time = time.time()
                                if current_time - last_publish_time >= publish_interval:
                                    # 创建并发布ROS LaserScan消息
                                    laser_msg = self.create_laserscan_msg()
                                    self.laser_pub.publish(laser_msg)

                                    # 创建并发布ROS IMU消息
                                    imu_msg = self.create_imu_msg()
                                    self.imu_pub.publish(imu_msg)

                                    last_publish_time = current_time
                                    self.last_publish_time = current_time

                                # 定期打印状态
                                current_time = time.time()
                                if current_time - self.last_status_time >= 5.0:
                                    valid_points = np.sum(self.distance_array < float('inf'))
                                    ax, ay, az, gx, gy, gz = self.imu_raw_data
                                    roll, pitch, yaw = self.imu_rpy_data
                                    print(f"✓ 已解析 {self.frames_parsed} 帧, 有效点: {valid_points}/{self.num_points}")
                                    print(
                                        f"  IMU原始数据 - 加速度: [{ax:.2f}, {ay:.2f}, {az:.2f}], 角速度: [{gx:.2f}, {gy:.2f}, {gz:.2f}]")
                                    print(f"  IMU欧拉角 - 滚转: {roll:.2f}, 俯仰: {pitch:.2f}, 偏航: {yaw:.2f}")
                                    print(
                                        f"  编码器 - A轮: {self.wheelA_distance:.1f}mm, B轮: {self.wheelB_distance:.1f}mm")
                                    print(f"  速度 - 线速度: {self.speed_linear:.3f}m/s")
                                    print(
                                        f"  里程计 - X: {self.odom_x:.3f}m, Y: {self.odom_y:.3f}m, Yaw: {degrees(self.odom_yaw):.2f}°")
                                    self.last_status_time = current_time
                            else:
                                self.frames_failed += 1
                                search_state = "SEARCHING"  # 重新搜索帧头
                                break

                # 检查发布状态 - 如果长时间没有发布，发布一次空消息保持连接
                current_time = time.time()
                if current_time - self.last_publish_time > 1.0 and self.frames_parsed > 0:
                    print("警告: 长时间未发布数据，发布保持消息")
                    laser_msg = self.create_laserscan_msg()
                    self.laser_pub.publish(laser_msg)

                    imu_msg = self.create_imu_msg()
                    self.imu_pub.publish(imu_msg)

                    # 发布里程计tf和odom消息
                    # self.publish_odom_tf()
                    self.publish_odom_msg()

                    self.last_publish_time = current_time

                # 短暂休眠
                time.sleep(0.001)

            except socket.timeout:
                # 超时是正常的，继续循环
                continue
            except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError) as e:
                print(f"连接错误: {e}")
                return False
            except Exception as e:
                print(f"处理数据时出错: {e}")
                return False

        return True

    def run(self):
        """主运行循环"""
        print(f"启动RPLIDAR C1 ROS发布器")
        print(f"目标主机: {self.host}:{self.port}")
        print(f"数据帧大小: {self.frame_size} 字节")

        self.last_status_time = time.time()
        self.last_publish_time = time.time()
        self.last_odom_update_time = time.time()

        while not rospy.is_shutdown():
            # 连接TCP服务器
            if not self.connect_tcp():
                print("连接失败，5秒后重试...")
                time.sleep(5)
                continue

            print("开始接收雷达、IMU和编码器数据...")
            print(f"期望帧头: 0x{self.expected_header:04X}")
            print(f"期望帧尾: 0x{self.expected_tail:04X}")

            # 处理数据流
            if self.process_data_stream():
                # 正常退出
                break
            else:
                # 连接出现问题，准备重连
                self.reconnect_count += 1
                print(f"连接断开，准备重新连接 (第{self.reconnect_count}次)...")

                # 清理资源
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                    self.socket = None

                # 等待后重试
                time.sleep(2)

        # 清理资源
        if self.socket:
            try:
                self.socket.close()
            except:
                pass

        print("ROS发布器已停止")

    def print_statistics(self):
        """打印统计信息"""
        if self.frames_parsed > 0:
            success_rate = self.frames_parsed / (self.frames_parsed + self.frames_failed) * 100
            print(f"\n最终统计:")
            print(f"  总接收字节: {self.bytes_received}")
            print(f"  解析帧数: {self.frames_parsed}")
            print(f"  失败帧数: {self.frames_failed}")
            print(f"  重连次数: {self.reconnect_count}")
            print(f"  成功率: {success_rate:.1f}%")


def main():
    # 虚拟机中的配置 - 连接到宿主机
    tcp_host = 'host.docker.internal'
    tcp_port = 12345

    parser = RPLidarC1ParserTCP(host=tcp_host, port=tcp_port)

    try:
        parser.run()
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        parser.print_statistics()


if __name__ == '__main__':
    main()