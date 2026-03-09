"""""""""
双端口版本的 serial_to_tcp
端口12345: 雷达数据下行 (只发送)
端口12346: 速度指令上行 (只接收)
"""""""""
import serial
import socket
import threading
import time
import struct


class DualPortSerialToTCP:
    def __init__(self, com_port, baudrate, radar_port=12345, command_port=12346):
        self.com_port = com_port
        self.baudrate = baudrate
        self.radar_port = radar_port
        self.command_port = command_port
        self.running = True

        # 雷达数据帧参数
        self.radar_frame_size = 572
        self.radar_header = 0xFF77
        self.radar_tail = 0x77EE

        # 统计信息
        self.valid_radar_frames_sent = 0
        self.speed_commands_received = 0
        self.invalid_frames_discarded = 0

    def start(self):
        try:
            # 打开串口
            self.ser = serial.Serial(
                port=self.com_port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1
            )
            print(f"串口 {self.com_port} 打开成功")

            # 启动雷达数据服务器（端口12345）
            radar_thread = threading.Thread(target=self.start_radar_server)
            radar_thread.daemon = True
            radar_thread.start()

            # 启动速度指令服务器（端口12346）
            command_thread = threading.Thread(target=self.start_command_server)
            command_thread.daemon = True
            command_thread.start()

            print(f"雷达数据服务器监听端口 {self.radar_port}")
            print(f"速度指令服务器监听端口 {self.command_port}")
            print("双端口服务已启动，按 Ctrl+C 停止")

            # 主线程等待
            while self.running:
                time.sleep(1)

        except Exception as e:
            print(f"错误: {e}")
        finally:
            self.stop()

    def start_radar_server(self):
        """启动雷达数据服务器（端口12345）"""
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('0.0.0.0', self.radar_port))
            server_socket.listen(5)  # 允许多个客户端连接

            while self.running:
                client_socket, addr = server_socket.accept()
                print(f"雷达数据客户端连接: {addr}")

                client_thread = threading.Thread(
                    target=self.handle_radar_client,
                    args=(client_socket,)
                )
                client_thread.daemon = True
                client_thread.start()

        except Exception as e:
            print(f"雷达服务器错误: {e}")

    def start_command_server(self):
        """启动速度指令服务器（端口12346）"""
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('0.0.0.0', self.command_port))
            server_socket.listen(5)  # 允许多个客户端连接

            while self.running:
                client_socket, addr = server_socket.accept()
                print(f"速度指令客户端连接: {addr}")

                client_thread = threading.Thread(
                    target=self.handle_command_client,
                    args=(client_socket,)
                )
                client_thread.daemon = True
                client_thread.start()

        except Exception as e:
            print(f"指令服务器错误: {e}")

    def is_valid_radar_frame(self, data):
        """验证雷达数据帧的有效性"""
        if len(data) != self.radar_frame_size:
            return False

        try:
            header = int.from_bytes(data[0:2], byteorder='big')
            tail = int.from_bytes(data[-2:], byteorder='big')
            return header == self.radar_header and tail == self.radar_tail
        except:
            return False

    def handle_radar_client(self, client_socket):
        """处理雷达数据客户端（只发送数据）"""
        try:
            print("开始发送雷达数据...")

            buffer = b''
            last_status_time = time.time()

            while self.running:
                # 从串口读取数据
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer += data

                    # 处理完整的雷达数据帧
                    while len(buffer) >= self.radar_frame_size:
                        frame_data = buffer[:self.radar_frame_size]

                        if self.is_valid_radar_frame(frame_data):
                            # 有效雷达帧，发送到TCP客户端
                            try:
                                client_socket.send(frame_data)
                                self.valid_radar_frames_sent += 1

                                # 控制输出频率
                                if self.valid_radar_frames_sent <= 5 or self.valid_radar_frames_sent % 50 == 0:
                                    print(f"✓ 发送雷达帧 #{self.valid_radar_frames_sent}")
                            except (BrokenPipeError, ConnectionResetError):
                                print("雷达客户端连接已断开")
                                return

                            # 移除已处理的有效帧
                            buffer = buffer[self.radar_frame_size:]
                        else:
                            # 无效帧，丢弃一个字节继续搜索
                            buffer = buffer[1:]
                            self.invalid_frames_discarded += 1

                # 定期输出状态信息
                current_time = time.time()
                if current_time - last_status_time >= 5.0:
                    print(
                        f"雷达数据状态: 缓冲区={len(buffer)}字节, 有效帧={self.valid_radar_frames_sent}, 无效帧={self.invalid_frames_discarded}")
                    last_status_time = current_time

                # 短暂休眠
                time.sleep(0.001)

        except Exception as e:
            print(f"雷达客户端处理错误: {e}")
        finally:
            try:
                client_socket.close()
            except:
                pass
            print("雷达客户端连接已关闭")

    def handle_command_client(self, client_socket):
        """处理速度指令客户端（只接收数据）"""
        try:
            print("开始接收速度指令...")

            while self.running:
                # 从TCP客户端接收速度指令
                try:
                    client_socket.settimeout(0.1)
                    command_data = client_socket.recv(1024)
                    if command_data:
                        # 直接写入串口
                        self.ser.write(command_data)
                        self.speed_commands_received += 1

                        # 解析并显示速度指令内容
                        if len(command_data) >= 12:  # 速度指令帧大小
                            try:
                                header = int.from_bytes(command_data[0:2], byteorder='big')
                                if header == 0xFF22:  # 速度指令帧头
                                    angular = struct.unpack('<f', command_data[2:6])[0]
                                    linear = struct.unpack('<f', command_data[6:10])[0]
                                    print(
                                        f"收到速度指令 #{self.speed_commands_received}: 角速度={angular:.1f}°/s, 线速度={linear:.2f}m/s")
                            except:
                                print(f"收到速度指令 #{self.speed_commands_received}: {len(command_data)} 字节")

                except socket.timeout:
                    continue
                except (BrokenPipeError, ConnectionResetError):
                    print("指令客户端连接已断开")
                    break

        except Exception as e:
            print(f"指令客户端处理错误: {e}")
        finally:
            try:
                client_socket.close()
            except:
                pass
            print("指令客户端连接已关闭")

    def stop(self):
        """停止服务"""
        self.running = False
        if hasattr(self, 'ser'):
            self.ser.close()


if __name__ == "__main__":
    # 配置参数
    COM_PORT = 'COM5'
    BAUD_RATE = 921600
    RADAR_PORT = 12345
    COMMAND_PORT = 12346

    forwarder = DualPortSerialToTCP(COM_PORT, BAUD_RATE, RADAR_PORT, COMMAND_PORT)

    try:
        forwarder.start()
    except KeyboardInterrupt:
        print("程序被用户中断")
        forwarder.stop()