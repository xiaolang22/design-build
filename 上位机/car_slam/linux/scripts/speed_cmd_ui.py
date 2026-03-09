#!/usr/bin/env python3
"""
摇杆速度控制UI
通过ROS发布 /cmd_vel 话题控制小车
摇杆松开时自动回中，速度归零
"""

import sys
import math
import rospy
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QGroupBox,
                             QSlider, QLineEdit)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont


class JoystickWidget(QWidget):
    """自定义摇杆控件"""

    # 定义信号：摇杆位置变化时发出
    positionChanged = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)

        # 摇杆参数
        self.joystick_radius = 20
        self.base_radius = 80
        self.center_x = 0
        self.center_y = 0
        self.joystick_x = 0
        self.joystick_y = 0

        # 鼠标状态
        self.mouse_pressed = False

        # 自动回中定时器
        self.return_timer = QTimer(self)
        self.return_timer.timeout.connect(self.return_to_center)
        self.return_interval = 100  # 100ms

    def paintEvent(self, event):
        """绘制摇杆"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 计算中心点
        self.center_x = self.width() / 2
        self.center_y = self.height() / 2

        # 绘制底座
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.setBrush(QBrush(QColor(200, 200, 200)))
        painter.drawEllipse(self.center_x - self.base_radius,
                            self.center_y - self.base_radius,
                            self.base_radius * 2, self.base_radius * 2)

        # 绘制坐标轴
        painter.setPen(QPen(QColor(150, 150, 150), 1))
        painter.drawLine(self.center_x - self.base_radius, self.center_y,
                         self.center_x + self.base_radius, self.center_y)
        painter.drawLine(self.center_x, self.center_y - self.base_radius,
                         self.center_x, self.center_y + self.base_radius)

        # 绘制摇杆
        painter.setPen(QPen(QColor(50, 50, 150), 2))
        painter.setBrush(QBrush(QColor(100, 100, 200)))

        joystick_center_x = self.center_x + self.joystick_x
        joystick_center_y = self.center_y + self.joystick_y

        painter.drawEllipse(joystick_center_x - self.joystick_radius,
                            joystick_center_y - self.joystick_radius,
                            self.joystick_radius * 2, self.joystick_radius * 2)

        # 显示坐标值
        painter.setPen(QPen(Qt.black))
        # 使用支持中文的字体
        font = QFont("Microsoft YaHei", 10)
        painter.setFont(font)
        x_percent = self.joystick_x / self.base_radius
        y_percent = -self.joystick_y / self.base_radius  # 反转Y轴，向上为正
        painter.drawText(10, 20, f"X: {x_percent:.2f}")
        painter.drawText(10, 40, f"Y: {y_percent:.2f}")

    def mousePressEvent(self, event):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            self.mouse_pressed = True
            self.update_joystick_position(event.pos())
            self.return_timer.stop()  # 停止自动回中

    def mouseMoveEvent(self, event):
        """鼠标移动事件"""
        if self.mouse_pressed:
            self.update_joystick_position(event.pos())

    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        if event.button() == Qt.LeftButton:
            self.mouse_pressed = False
            # 启动自动回中
            self.return_timer.start(self.return_interval)

    def update_joystick_position(self, pos):
        """更新摇杆位置"""
        # 计算相对于中心的位置
        dx = pos.x() - self.center_x
        dy = pos.y() - self.center_y

        # 限制在圆形范围内
        distance = math.sqrt(dx * dx + dy * dy)
        if distance > self.base_radius:
            dx = dx * self.base_radius / distance
            dy = dy * self.base_radius / distance

        self.joystick_x = dx
        self.joystick_y = dy

        # 发出位置变化信号
        x_percent = dx / self.base_radius
        y_percent = -dy / self.base_radius  # 反转Y轴，向上为正
        self.positionChanged.emit(x_percent, y_percent)

        self.update()

    def return_to_center(self):
        """自动回中"""
        if not self.mouse_pressed:
            # 逐步回中
            speed = 0.2
            dx = -self.joystick_x * speed
            dy = -self.joystick_y * speed

            self.joystick_x += dx
            self.joystick_y += dy

            # 计算当前位置百分比
            x_percent = self.joystick_x / self.base_radius
            y_percent = -self.joystick_y / self.base_radius

            # 发出位置变化信号
            self.positionChanged.emit(x_percent, y_percent)

            self.update()

            # 如果接近中心，停止定时器
            if abs(self.joystick_x) < 1 and abs(self.joystick_y) < 1:
                self.joystick_x = 0
                self.joystick_y = 0
                self.return_timer.stop()


class JoystickControlUI(QMainWindow):
    """摇杆控制主界面"""

    def __init__(self):
        super().__init__()

        # ROS初始化
        rospy.init_node('joystick_control_ui', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 速度参数
        self.max_linear_speed = 0.5  # 最大线速度 m/s
        self.max_angular_speed = 1.0  # 最大角速度 rad/s

        # 当前速度
        self.current_linear = 0.0
        self.current_angular = 0.0

        # 发布定时器
        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.publish_cmd_vel)
        self.publish_interval = 50  # 50ms, 20Hz

        self.init_ui()

    def init_ui(self):
        """初始化UI"""
        self.setWindowTitle("Joystick Speed Control")
        self.setGeometry(100, 100, 600, 500)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 摇杆控制组
        joystick_group = QGroupBox("Joystick Control")
        joystick_layout = QVBoxLayout()

        # 摇杆控件
        self.joystick = JoystickWidget()
        self.joystick.positionChanged.connect(self.on_joystick_position_changed)
        joystick_layout.addWidget(self.joystick)

        # 摇杆说明
        joystick_help = QLabel(
            "Drag the joystick to control speed\nUp: Forward, Down: Backward, Left: Turn Left, Right: Turn Right\nRelease to auto-center")
        joystick_help.setAlignment(Qt.AlignCenter)
        joystick_layout.addWidget(joystick_help)

        joystick_group.setLayout(joystick_layout)
        layout.addWidget(joystick_group)

        # 速度显示组
        speed_group = QGroupBox("Speed Display")
        speed_layout = QHBoxLayout()

        self.linear_label = QLabel("Linear: 0.00 m/s")
        self.angular_label = QLabel("Angular: 0.00 rad/s")

        speed_layout.addWidget(self.linear_label)
        speed_layout.addWidget(self.angular_label)

        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

        # 参数设置组
        params_group = QGroupBox("Parameter Settings")
        params_layout = QVBoxLayout()

        # 最大线速度设置
        linear_layout = QHBoxLayout()
        linear_layout.addWidget(QLabel("Max Linear:"))
        self.linear_slider = QSlider(Qt.Horizontal)
        self.linear_slider.setRange(10, 100)  # 0.1 - 1.0 m/s
        self.linear_slider.setValue(int(self.max_linear_speed * 100))
        self.linear_slider.valueChanged.connect(self.on_linear_slider_changed)
        linear_layout.addWidget(self.linear_slider)

        self.linear_value = QLineEdit(f"{self.max_linear_speed:.2f}")
        self.linear_value.setMaximumWidth(50)
        self.linear_value.textChanged.connect(self.on_linear_text_changed)
        linear_layout.addWidget(self.linear_value)
        linear_layout.addWidget(QLabel("m/s"))

        params_layout.addLayout(linear_layout)

        # 最大角速度设置
        angular_layout = QHBoxLayout()
        angular_layout.addWidget(QLabel("Max Angular:"))
        self.angular_slider = QSlider(Qt.Horizontal)
        self.angular_slider.setRange(10, 200)  # 0.1 - 2.0 rad/s
        self.angular_slider.setValue(int(self.max_angular_speed * 100))
        self.angular_slider.valueChanged.connect(self.on_angular_slider_changed)
        angular_layout.addWidget(self.angular_slider)

        self.angular_value = QLineEdit(f"{self.max_angular_speed:.2f}")
        self.angular_value.setMaximumWidth(50)
        self.angular_value.textChanged.connect(self.on_angular_text_changed)
        angular_layout.addWidget(self.angular_value)
        angular_layout.addWidget(QLabel("rad/s"))

        params_layout.addLayout(angular_layout)

        params_group.setLayout(params_layout)
        layout.addWidget(params_group)

        # 控制按钮组
        button_group = QGroupBox("Control")
        button_layout = QHBoxLayout()

        self.start_btn = QPushButton("Start Publishing")
        self.start_btn.clicked.connect(self.start_publishing)
        button_layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_robot)
        button_layout.addWidget(self.stop_btn)

        button_group.setLayout(button_layout)
        layout.addWidget(button_group)

        # 状态显示
        self.status_label = QLabel("Status: Not Publishing")
        layout.addWidget(self.status_label)

        # 启动发布定时器
        self.publish_timer.start(self.publish_interval)

        # 设置全局字体
        self.set_global_font()

    def set_global_font(self):
        """设置全局字体，解决中文乱码问题"""
        # 尝试使用系统中可用的字体
        font_families = [
            "Microsoft YaHei",  # Windows 中文
            "SimHei",  # Windows 黑体
            "WenQuanYi Micro Hei",  # Linux 中文
            "Arial Unicode MS",  # 跨平台
            "DejaVu Sans",  # 跨平台
            "Arial"  # 英文
        ]

        # 创建字体
        font = QFont()
        font.setPointSize(10)

        # 尝试设置字体
        for font_family in font_families:
            font.setFamily(font_family)
            if font.exactMatch():
                print(f"Using font: {font_family}")
                break

        # 应用字体到所有控件
        self.setFont(font)
        QApplication.setFont(font)

    def on_joystick_position_changed(self, x_percent, y_percent):
        """摇杆位置变化回调"""
        # 计算速度值
        self.current_linear = y_percent * self.max_linear_speed
        self.current_angular = x_percent * self.max_angular_speed

        # 更新显示
        self.linear_label.setText(f"Linear: {self.current_linear:.2f} m/s")
        self.angular_label.setText(f"Angular: {self.current_angular:.2f} rad/s")

    def publish_cmd_vel(self):
        """发布速度指令"""
        if hasattr(self, 'cmd_vel_pub'):
            twist_msg = Twist()
            twist_msg.linear.x = self.current_linear
            twist_msg.angular.z = self.current_angular

            self.cmd_vel_pub.publish(twist_msg)

    def start_publishing(self):
        """开始发布"""
        self.status_label.setText("Status: Publishing...")
        self.start_btn.setEnabled(False)

    def stop_robot(self):
        """停止机器人"""
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.linear_label.setText("Linear: 0.00 m/s")
        self.angular_label.setText("Angular: 0.00 rad/s")
        self.status_label.setText("Status: Stopped")
        self.start_btn.setEnabled(True)

    def on_linear_slider_changed(self, value):
        """线速度滑块变化"""
        self.max_linear_speed = value / 100.0
        self.linear_value.setText(f"{self.max_linear_speed:.2f}")

    def on_angular_slider_changed(self, value):
        """角速度滑块变化"""
        self.max_angular_speed = value / 100.0
        self.angular_value.setText(f"{self.max_angular_speed:.2f}")

    def on_linear_text_changed(self, text):
        """线速度文本变化"""
        try:
            value = float(text)
            if 0.1 <= value <= 1.0:
                self.max_linear_speed = value
                self.linear_slider.setValue(int(value * 100))
        except ValueError:
            pass

    def on_angular_text_changed(self, text):
        """角速度文本变化"""
        try:
            value = float(text)
            if 0.1 <= value <= 2.0:
                self.max_angular_speed = value
                self.angular_slider.setValue(int(value * 100))
        except ValueError:
            pass

    def closeEvent(self, event):
        """窗口关闭事件"""
        # 停止机器人
        self.stop_robot()
        # 发布最后一次停止指令
        self.publish_cmd_vel()
        event.accept()


def main():
    app = QApplication(sys.argv)

    # 设置应用样式
    app.setStyle('Fusion')

    window = JoystickControlUI()
    window.show()

    # ROS循环处理
    timer = QTimer()
    timer.timeout.connect(lambda: None)  # 空函数，只是为了让ROS能够处理消息
    timer.start(100)  # 100ms

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()