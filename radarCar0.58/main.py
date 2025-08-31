from maze_utils import load_edges, draw_maze
from robot_utils import Robot, simulate_lidar_scan
from explore2 import explore_maze  # 新增：导入探索主函数
import matplotlib.pyplot as plt
import numpy as np
import sys
import random
import subprocess
import json
from PyQt5.QtCore import Qt, QPropertyAnimation, QEasingCurve, QSize, QPoint, QRect
from PyQt5.QtGui import QPixmap, QIcon, QFont, QFontDatabase, QPainter, QColor, QBrush, QPen, QLinearGradient, QConicalGradient, QPainterPath
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QPushButton, QVBoxLayout, QLabel, 
                           QMessageBox, QHBoxLayout, QSizePolicy, QGraphicsDropShadowEffect, QGraphicsOpacityEffect)
from PyQt5.QtWidgets import QComboBox, QDialog, QLineEdit, QFrame
from PyQt5.QtWidgets import QFileDialog

# ===== 主题配色方案 - 更亮的颜色 =====
light_qss = """
QWidget {
    background-color: #f8faff;  /* 更亮的浅蓝色背景 */
    color: #333;  /* 更深的文字颜色，增加对比度 */
    font-family: "Comic Sans MS", "楷体", "微软雅黑";  /* 可爱风格字体 */
}
QPushButton {
    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffb0e0, stop:1 #ff7dcb);  /* 更亮的粉色渐变 */
    color: #fff;
    border-radius: 20px;  /* 更圆的按钮 */
    font-size: 22px;
    font-weight: bold;
    padding: 12px 24px;
    border: none;
    border-bottom: 4px solid #ff5fb8;  /* 更亮的3D效果 */
}
QPushButton:hover {
    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffb5e3, stop:1 #ff89d0);
    transform: translateY(-2px);
}
QPushButton:pressed {
    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #e55bb1, stop:1 #ff89d0);
    padding-top: 14px;
    padding-bottom: 10px;
    border-bottom: 2px solid #e456ae;
}
QLabel {
    color: #444;
    font-family: "Comic Sans MS", "楷体", "微软雅黑";  /* 可爱风格字体 */
}
QComboBox {
    border-radius: 15px;
    padding: 6px 12px;
    background-color: #ffffff;  /* 纯白色背景 */
    border: 3px solid #ffb0e0;  /* 更亮的粉色边框 */
    font-family: "Comic Sans MS", "楷体", "微软雅黑";
    color: #222;  /* 更深的文字颜色增加对比度 */
}
QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: right;
    width: 24px;
    border-left: 3px solid #ff9ed8;
    border-top-right-radius: 13px;
    border-bottom-right-radius: 13px;
}
QComboBox QAbstractItemView {
    border: 3px solid #ffb0e0;
    border-radius: 10px;
    background-color: #fffdfe;  /* 更亮的背景 */
    color: #222;  /* 更深的文字颜色增加对比度 */
}
QFrame {
    border-radius: 20px;
}
"""

dark_qss = """
QWidget {
    background-color: #343466;  /* 更亮的深色背景 */
    color: #ffffff;  /* 纯白文字提高对比度 */
    font-family: "Comic Sans MS", "楷体", "微软雅黑";  /* 可爱风格字体 */
}
QPushButton {
    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #a792d9, stop:1 #7a6bde);  /* 更亮的紫色系渐变 */
    color: #fff;
    border-radius: 20px;  /* 更圆的按钮 */
    font-size: 22px;
    font-weight: bold;
    padding: 12px 24px;
    border: none;
    border-bottom: 4px solid #6a5dcf;  /* 更亮的3D效果 */
}
QPushButton:hover {
    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #a48fd6, stop:1 #7a68dd);
}
QPushButton:pressed {
    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #5d4eb3, stop:1 #7a68dd);
    padding-top: 14px;
    padding-bottom: 10px;
    border-bottom: 2px solid #5d4eb3;
}
QLabel {
    color: #e6e6e6;
    font-family: "Comic Sans MS", "楷体", "微软雅黑";
}
QComboBox {
    border-radius: 15px;
    padding: 6px 12px;
    background-color: #454580;  /* 更亮的背景 */
    color: #ffffff;  /* 纯白文字 */
    border: 3px solid #a792d9;  /* 更亮的紫色边框 */
    font-family: "Comic Sans MS", "楷体", "微软雅黑";
}
QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: right;
    width: 24px;
    border-left: 3px solid #8e78be;
    border-top-right-radius: 13px;
    border-bottom-right-radius: 13px;
}
QComboBox QAbstractItemView {
    background-color: #383861;
    border: 3px solid #8e78be;
    border-radius: 10px;
}
QFrame {
    border-radius: 20px;
    background-color: rgba(60, 60, 120, 0.65);  /* 更亮的半透明紫色 */
}
/* 滚动条美化 */
QScrollBar:vertical {
    border: none;
    background: #383861;
    width: 12px;
    margin: 15px 0 15px 0;
    border-radius: 6px;
}
QScrollBar::handle:vertical {
    background: #8e78be;
    min-height: 20px;
    border-radius: 6px;
}
"""

def draw_robot_and_scan(ax, robot, scan_data):
    """
    在给定的坐标系上绘制机器人和雷达扫描数据
    """
    # 绘制机器人位置 (一个蓝色的圆点) 和朝向 (一条红线)
    ax.plot(robot.x, robot.y, 'bo', markersize=8, label='Robot')
    arrow_len = 2.0
    ax.arrow(robot.x, robot.y, 
             arrow_len * np.cos(robot.theta), 
             arrow_len * np.sin(robot.theta), 
             head_width=0.5, head_length=0.7, fc='r', ec='r')

    # 绘制雷达扫描点云
    scan_points_x = []
    scan_points_y = []
    for angle, dist in scan_data:
        scan_points_x.append(robot.x + dist * np.cos(angle))
        scan_points_y.append(robot.y + dist * np.sin(angle))
    
    ax.scatter(scan_points_x, scan_points_y, c='g', s=5, label='Lidar Points')

class AnimatedButton(QPushButton):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._animation = QPropertyAnimation(self, b"geometry")
        self._animation.setDuration(120)
        self._animation.setEasingCurve(QEasingCurve.InOutQuad)
        self._orig_geometry = None
        
        # Add shadow effect
        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(20)
        shadow.setColor(QColor(0, 0, 0, 80))
        shadow.setOffset(0, 4)
        self.setGraphicsEffect(shadow)
        
        # Override style
        self.setStyleSheet(self.styleSheet() + """
            text-align: center;
            font-family: 'Segoe UI', Arial, sans-serif;
        """)
        
    def mousePressEvent(self, event):
        self._orig_geometry = self.geometry()
        shrink = self._orig_geometry.adjusted(4, 4, -4, -4)
        self._animation.stop()
        self._animation.setStartValue(self._orig_geometry)
        self._animation.setEndValue(shrink)
        self._animation.start()
        
        # Enhance shadow for pressed state
        shadow = self.graphicsEffect()
        if shadow:
            shadow.setBlurRadius(10)
            shadow.setOffset(0, 2)
        
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if self._orig_geometry:
            self._animation.stop()
            self._animation.setStartValue(self.geometry())
            self._animation.setEndValue(self._orig_geometry)
            self._animation.start()
            
            # Restore shadow
            shadow = self.graphicsEffect()
            if shadow:
                shadow.setBlurRadius(20)
                shadow.setOffset(0, 4)
                
        super().mouseReleaseEvent(event)
        
    def enterEvent(self, event):
        # Brighten on hover
        shadow = self.graphicsEffect()
        if shadow:
            shadow.setBlurRadius(25)
            shadow.setOffset(0, 5)
        super().enterEvent(event)
        
    def leaveEvent(self, event):
        # Restore shadow
        shadow = self.graphicsEffect()
        if shadow:
            shadow.setBlurRadius(20)
            shadow.setOffset(0, 4)
        super().leaveEvent(event)

class WelcomeWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.is_dark = False
        self.setWindowTitle("雷达小车探索系统")
        self.setGeometry(400, 200, 1600, 1200)
        self.opacity_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self.opacity_effect)
        
        # 加载自定义字体
        self.load_fonts()
        
        # 创建动画背景
        self.cartoon_elements = []
        self.init_cartoon_elements()
        
        self.init_ui()
        self.apply_theme()
        
    def load_fonts(self):
        """加载自定义字体"""
        # 加载可爱风格的字体
        try:
            # 尝试加载可爱风格的字体
            QFontDatabase.addApplicationFont("fonts/comic.ttf")
            QFontDatabase.addApplicationFont("fonts/rounded.ttf")
            QFontDatabase.addApplicationFont("fonts/kawaii.ttf")  # 可爱风格字体
            QFontDatabase.addApplicationFont("fonts/bubblegum.ttf")  # 泡泡糖风格字体
            
            # 设置应用程序默认字体
            cute_font = QFont("Bubblegum Sans", 12)  # 先尝试泡泡糖字体
            if "Bubblegum Sans" not in cute_font.family():
                cute_font = QFont("Comic Sans MS", 12)  # 退回到Comic Sans
            QApplication.setFont(cute_font)
        except:
            # 如果字体不存在，使用系统可爱风格字体
            cute_font = QFont("Comic Sans MS", 12)
            QApplication.setFont(cute_font)
            
    def init_cartoon_elements(self):
        """初始化卡通元素"""
        # 加载卡通图像或创建默认元素
        # 机器人图标
        self.robot_icon = QPixmap("robot_cartoon.png")
        if self.robot_icon.isNull():
            self.robot_icon = self.create_robot_icon()
            
        # 雷达图标
        self.radar_icon = QPixmap("radar_cartoon.png")
        if self.radar_icon.isNull():
            self.radar_icon = self.create_radar_icon()
            
        # 额外的装饰元素
        self.cloud_icon = QPixmap("cloud_cartoon.png")
        self.star_icon = QPixmap("star_cartoon.png")
        self.gear_icon = QPixmap("gear_cartoon.png")
        
        # 动画元素列表
        self.animated_elements = []
        
    def create_robot_icon(self, size=120):
        """如果没有机器人图标，创建一个简单的卡通机器人"""
        icon = QPixmap(size, size)
        icon.fill(Qt.transparent)
        
        painter = QPainter(icon)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 机器人头部 - 蓝色圆形
        painter.setBrush(QBrush(QColor("#4a90e2")))
        painter.setPen(QPen(QColor("#2c3e50"), 2))
        painter.drawEllipse(10, 10, size-20, size-20)
        
        # 眼睛 - 两个白色圆圈带黑点
        painter.setBrush(QBrush(Qt.white))
        painter.drawEllipse(size//4, size//3, size//5, size//5)
        painter.drawEllipse(size//2, size//3, size//5, size//5)
        
        painter.setBrush(QBrush(Qt.black))
        painter.drawEllipse(size//4 + size//15, size//3 + size//15, size//10, size//10)
        painter.drawEllipse(size//2 + size//15, size//3 + size//15, size//10, size//10)
        
        # 嘴巴 - 微笑线
        painter.setPen(QPen(QColor("#2c3e50"), 3))
        painter.drawArc(size//4, size//2, size//2, size//4, 0, 180 * 16)
        
        # 天线
        painter.drawLine(size//2, 10, size//2, 0)
        painter.drawEllipse(size//2 - 5, 0, 10, 10)
        
        painter.end()
        return icon
        
    def create_radar_icon(self, size=120):
        """如果没有雷达图标，创建一个简单的雷达扫描"""
        icon = QPixmap(size, size)
        icon.fill(Qt.transparent)
        
        painter = QPainter(icon)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 雷达外圈
        painter.setBrush(QBrush(QColor(0, 200, 0, 30)))
        painter.setPen(QPen(QColor("#27ae60"), 2))
        painter.drawEllipse(10, 10, size-20, size-20)
        
        # 雷达中圈
        painter.drawEllipse(size//4, size//4, size//2, size//2)
        
        # 雷达内圈
        painter.drawEllipse(size//2 - size//6, size//2 - size//6, size//3, size//3)
        
        # 雷达十字线
        painter.drawLine(size//2, 10, size//2, size-10)
        painter.drawLine(10, size//2, size-10, size//2)
        
        # 雷达扫描区
        gradient = QConicalGradient(size//2, size//2, 0)
        gradient.setColorAt(0.0, QColor(0, 255, 0, 150))
        gradient.setColorAt(0.2, QColor(0, 255, 0, 100))
        gradient.setColorAt(0.5, QColor(0, 255, 0, 0))
        gradient.setColorAt(0.9, QColor(0, 255, 0, 0))
        gradient.setColorAt(1.0, QColor(0, 255, 0, 150))
        
        painter.setBrush(QBrush(gradient))
        painter.setPen(Qt.NoPen)
        painter.drawPie(10, 10, size-20, size-20, 0, 90 * 16)
        
        # 中心点
        painter.setBrush(QBrush(QColor("#27ae60")))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(size//2 - 5, size//2 - 5, 10, 10)
        
        painter.end()
        return icon

    def init_ui(self):
        central_widget = QWidget()
        self.main_layout = QVBoxLayout()
        self.main_layout.setContentsMargins(60, 50, 60, 50)
        self.main_layout.setSpacing(36)
        
        # 添加一个装饰性的顶部框架
        top_frame = QFrame()
        top_frame.setFrameShape(QFrame.StyledPanel)
        top_frame.setStyleSheet("border-radius: 25px;")
        top_frame_layout = QVBoxLayout(top_frame)
        
        self.label_stretch_top = self.main_layout.addStretch(2)

        # ===== 顶部LOGO图片 =====
        header_layout = QHBoxLayout()
        
        # 左侧机器人卡通图
        robot_label = QLabel()
        if not self.robot_icon.isNull():
            robot_label.setPixmap(self.robot_icon.scaledToHeight(120, Qt.SmoothTransformation))
        robot_label.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(robot_label)
        
        # 中间LOGO和标题
        center_layout = QVBoxLayout()
        logo_label = QLabel()
        logo_pix = QPixmap("logo.png")
        if logo_pix.isNull():
            logo_label.setText("🚗📡")
            font = QFont("Segoe UI Emoji", 48)
            logo_label.setFont(font)
            logo_label.setStyleSheet("color: #4a90e2;")
        else:
            logo_label.setPixmap(logo_pix.scaledToHeight(120, Qt.SmoothTransformation))
        logo_label.setAlignment(Qt.AlignCenter)
        center_layout.addWidget(logo_label)
        
        title_label = QLabel("雷达小车探索系统")
        # 使用可爱风格的字体
        title_font = QFont("Comic Sans MS", 36, QFont.Bold)
        title_label.setFont(title_font)
        title_label.setStyleSheet("font-weight: bold; margin-bottom: 18px; color: #ff6ec4; text-shadow: 2px 2px 4px rgba(255,192,203,0.4);")
        title_label.setAlignment(Qt.AlignCenter)
        center_layout.addWidget(title_label)
        header_layout.addLayout(center_layout)
        
        # 右侧雷达卡通图
        radar_label = QLabel()
        if not self.radar_icon.isNull():
            radar_label.setPixmap(self.radar_icon.scaledToHeight(120, Qt.SmoothTransformation))
        radar_label.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(radar_label)
        
        self.main_layout.addLayout(header_layout)
        
        # 欢迎标语
        welcome_label = QLabel("探索未知世界，定位与导航的奇妙旅程！")
        welcome_font = QFont("Comic Sans MS", 22)
        welcome_label.setFont(welcome_font)
        welcome_label.setStyleSheet("font-style: italic; margin-top: 10px; margin-bottom: 30px; color: #8e78be; letter-spacing: 1px;")
        welcome_label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(welcome_label)

        # 创建一个装饰性面板
        buttons_frame = QFrame()
        buttons_frame.setStyleSheet("""
            QFrame {
                border-radius: 30px;
                background-color: rgba(255, 255, 255, 0.85);  /* 更亮更透明的背景 */
                border: 2px solid rgba(255, 220, 240, 0.7);   /* 淡粉色边框 */
            }
        """)
        
        buttons_layout = QVBoxLayout(buttons_frame)
        buttons_layout.setContentsMargins(40, 30, 40, 30)
        buttons_layout.setSpacing(30)
        
        # 添加功能按钮标题
        functions_label = QLabel("功能选择")
        functions_label.setAlignment(Qt.AlignCenter)
        functions_font = QFont("Comic Sans MS", 24, QFont.Bold)
        functions_label.setFont(functions_font)
        functions_label.setStyleSheet("color: #ff6ec4; margin-bottom: 20px; text-shadow: 1px 1px 2px rgba(255,192,203,0.4);")
        buttons_layout.addWidget(functions_label)
        
        # 添加主功能按钮
        self.btn_layout = QHBoxLayout()
        self.btn_layout.setSpacing(50)
        self.btn_layout.setContentsMargins(30, 10, 30, 10)
        
        btn_slam = AnimatedButton("SLAM扫描")
        btn_plan = AnimatedButton("路径规划")
        btn_draw_map = AnimatedButton("手绘地图")  # 新增手绘地图按钮
        
        # ===== 按钮加图标 =====
        radar_icon = QIcon("radar_icon.png")
        plan_icon = QIcon("plan_icon.png")
        draw_map_icon = QIcon("draw_map_icon.png")  # 假设有这个图标
        
        btn_slam.setIcon(radar_icon)
        btn_plan.setIcon(plan_icon)
        btn_draw_map.setIcon(draw_map_icon if not QPixmap("draw_map_icon.png").isNull() else radar_icon)
        
        # 设置图标大小
        btn_slam.setIconSize(QSize(48, 48))
        btn_plan.setIconSize(QSize(48, 48))
        btn_draw_map.setIconSize(QSize(48, 48))
        
        for btn in (btn_slam, btn_plan, btn_draw_map):
            btn.setMinimumHeight(120)
            btn.setMaximumHeight(120)
            btn.setMinimumWidth(220)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            # 使用大字体
            btn_font = QFont("微软雅黑", 18, QFont.Bold)
            btn.setFont(btn_font)
            
        self.btn_layout.addStretch(1)
        self.btn_layout.addWidget(btn_slam)
        self.btn_layout.addWidget(btn_plan)
        self.btn_layout.addWidget(btn_draw_map)  # 添加手绘地图按钮
        self.btn_layout.addStretch(1)
        
        buttons_layout.addLayout(self.btn_layout)
        self.main_layout.addWidget(buttons_frame)

        # 创建底部设置面板
        settings_frame = QFrame()
        settings_frame.setStyleSheet("""
            QFrame {
                border-radius: 20px;
                background-color: rgba(250, 245, 255, 0.8);  /* 更亮的淡紫色背景 */
                border: 2px solid rgba(230, 210, 255, 0.7);  /* 淡紫色边框 */
                margin-top: 20px;
            }
        """)
        settings_layout = QVBoxLayout(settings_frame)
        settings_layout.setContentsMargins(30, 20, 30, 20)
        
        # 设置标题
        settings_title = QLabel("界面设置")
        settings_title.setAlignment(Qt.AlignCenter)
        settings_title_font = QFont("Comic Sans MS", 20)
        settings_title.setFont(settings_title_font)
        settings_title.setStyleSheet("color: #a792d9; margin-bottom: 10px; text-shadow: 1px 1px 2px rgba(230,210,255,0.4);")
        settings_layout.addWidget(settings_title)
        
        # 主题设置
        self.theme_layout = QHBoxLayout()
        self.theme_layout.setSpacing(20)
        self.theme_layout.setContentsMargins(10, 5, 10, 5)
        
        theme_label = QLabel("主题模式：")
        theme_label_font = QFont("微软雅黑", 16)
        theme_label.setFont(theme_label_font)
        theme_label.setStyleSheet("color: #333;")
        
        self.theme_combo = QComboBox()
        theme_combo_font = QFont("微软雅黑", 14)
        self.theme_combo.setFont(theme_combo_font)
        self.theme_combo.addItems(["浅色模式", "暗色模式", "高对比度模式"])
        self.theme_combo.setCurrentIndex(1 if self.is_dark else 0)
        self.theme_combo.setFixedHeight(45)
        self.theme_combo.setMinimumWidth(180)
        self.theme_combo.setStyleSheet("""
            QComboBox {
                padding-left: 15px;
            }
        """)
        self.theme_combo.currentIndexChanged.connect(self.on_theme_selected)
        
        self.btn_theme = AnimatedButton("切换主题")
        theme_btn_font = QFont("微软雅黑", 14, QFont.Bold)
        self.btn_theme.setFont(theme_btn_font)
        self.btn_theme.setFixedHeight(45)
        self.btn_theme.setMinimumWidth(150)
        self.btn_theme.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        
        # 主题按钮加图标
        theme_icon = QIcon("theme_icon.png")
        self.btn_theme.setIcon(theme_icon)
        self.btn_theme.setIconSize(QSize(24, 24))
        self.btn_theme.clicked.connect(self.on_theme_button_clicked)
        
        self.theme_layout.addStretch(1)
        self.theme_layout.addWidget(theme_label, alignment=Qt.AlignVCenter)
        self.theme_layout.addWidget(self.theme_combo, alignment=Qt.AlignVCenter)
        self.theme_layout.addWidget(self.btn_theme, alignment=Qt.AlignVCenter)
        self.theme_layout.addStretch(1)
        
        settings_layout.addLayout(self.theme_layout)
        self.main_layout.addWidget(settings_frame)

        # 创建底部工具栏
        tools_layout = QHBoxLayout()
        tools_layout.setContentsMargins(20, 30, 20, 10)
        tools_layout.setSpacing(20)
        
        # 增加帮助按钮
        btn_help = AnimatedButton("帮助与支持")
        btn_help.setAccessibleName("帮助按钮")
        btn_help.setAccessibleDescription("点击以获取帮助信息")
        btn_help.setFocusPolicy(Qt.StrongFocus)
        btn_help.setFixedHeight(50)
        btn_help.setMinimumWidth(180)
        help_btn_font = QFont("微软雅黑", 14)
        btn_help.setFont(help_btn_font)
        
        # 添加帮助图标
        btn_help.setIcon(QIcon("help_icon.png"))
        btn_help.setIconSize(QSize(24, 24))
        
        btn_help.clicked.connect(self.show_help)
        
        # 添加版本信息标签
        version_label = QLabel("雷达小车探索系统 v1.0.2")
        version_font = QFont("微软雅黑", 12)
        version_label.setFont(version_font)
        version_label.setStyleSheet("color: #666; font-style: italic;")
        
        tools_layout.addWidget(version_label, alignment=Qt.AlignLeft | Qt.AlignVCenter)
        tools_layout.addStretch(1)
        tools_layout.addWidget(btn_help, alignment=Qt.AlignRight)
        
        self.main_layout.addLayout(tools_layout)
        self.label_stretch_bottom = self.main_layout.addStretch(1)

        central_widget.setLayout(self.main_layout)
        self.setCentralWidget(central_widget)
        
        # 设置窗口背景 - 更亮的背景
        self.setStyleSheet(self.styleSheet() + """
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, 
                                           stop:0 #f8faff, stop:0.3 #fff8ff, stop:0.7 #fff8fa, stop:1 #f8ffff);
            }
        """)
        
        # 添加悬浮云朵装饰
        self.add_floating_decorations()

        # ===== 设置可爱渐变背景 - 更亮的背景 =====
        self.setStyleSheet(self.styleSheet() + """
            QWidget#WelcomeWindow {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, 
                                           stop:0 #fffaf0, stop:0.3 #fff0f8, 
                                           stop:0.6 #f0f8ff, stop:1 #f8fff8);
            }
        """)
        self.setObjectName("WelcomeWindow")

        btn_slam.clicked.connect(self.run_slam)
        btn_plan.clicked.connect(self.run_plan)
        btn_draw_map.clicked.connect(self.run_draw_map)

    def resizeEvent(self, event):
        w = self.width()
        h = self.height()
        # 主布局边距和间距自适应
        margin = max(20, int(min(w, h) * 0.06))  # 边距随窗口缩放
        spacing = max(12, int(min(w, h) * 0.04)) # 主区块间距
        self.main_layout.setContentsMargins(margin, margin, margin, margin)
        self.main_layout.setSpacing(spacing)
        # 按钮区和主题区间距自适应
        btn_spacing = max(10, int(w * 0.03))
        self.btn_layout.setSpacing(btn_spacing)
        theme_spacing = max(6, int(w * 0.015))
        self.theme_layout.setSpacing(theme_spacing)

        # ====== 动态放大按钮和主题区控件 ======
        btn_height = max(48, int(h * 0.08))
        btn_font = max(18, int(h * 0.035))
        btn_width = max(120, int(w * 0.18))
        # 遍历按钮区控件
        for i in range(self.btn_layout.count()):
            item = self.btn_layout.itemAt(i)
            widget = item.widget()
            if isinstance(widget, AnimatedButton):
                widget.setMinimumHeight(btn_height)
                widget.setMaximumHeight(btn_height)
                widget.setMinimumWidth(btn_width)
                widget.setStyleSheet(f"font-size: {btn_font}px;")
        # 主题区控件
        combo_height = max(36, int(h * 0.06))
        combo_font = max(15, int(h * 0.025))
        combo_width = max(100, int(w * 0.13))
        self.theme_combo.setFixedHeight(combo_height)
        self.theme_combo.setMinimumWidth(combo_width)
        self.theme_combo.setStyleSheet(f"font-size: {combo_font}px;")
        self.btn_theme.setFixedHeight(combo_height)
        self.btn_theme.setMinimumWidth(combo_width)
        self.btn_theme.setStyleSheet(f"font-size: {combo_font}px;")
        super().resizeEvent(event)

    def on_theme_selected(self, idx):
        # 任何主题切换都立即应用，无需判断
        self.apply_theme()

    def on_theme_button_clicked(self):
        # 按钮切换主题，并同步下拉框
        want_dark = not self.is_dark
        self.fade_theme(lambda: self.set_theme(want_dark))
        self.theme_combo.setCurrentIndex(1 if want_dark else 0)

    def set_theme(self, dark):
        self.is_dark = dark
        self.apply_theme()

    def apply_theme(self):
        # 高对比度主题：黑底黄字
        if self.theme_combo.currentIndex() == 2:
            self.setStyleSheet("""
            QWidget { background: #000; color: #FFD600; }
            QPushButton { 
                background: #FFD600; 
                color: #000; 
                font-weight: bold; 
                border-radius: 16px; 
                font-size: 22px;
                font-family: "微软雅黑"; 
                padding: 12px 24px;
            }
            QPushButton:hover { 
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #FFF176, stop:1 #FFD600); 
                color: #000; 
            }
            QPushButton:pressed { 
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #E6C200, stop:1 #FFD600); 
                padding-top: 14px;
                padding-bottom: 10px;
            }
            QLabel { color: #FFD600; }
            QComboBox { 
                background: #222; 
                color: #FFD600; 
                border: 2px solid #FFD600; 
                border-radius: 10px; 
                font-size: 16px;
                padding: 6px 12px; 
            }
            QComboBox::drop-down { 
                border-left: 2px solid #FFD600; 
            }
            QComboBox QAbstractItemView { 
                background: #222; 
                color: #FFD600; 
                border: 2px solid #FFD600;
            }
            QMessageBox { 
                background: #000; 
                color: #FFD600; 
            }
            QFrame { 
                background: #111; 
                border: 2px solid #FFD600;
            }
            """)
        elif self.is_dark:
            self.setStyleSheet(dark_qss)
        else:
            self.setStyleSheet(light_qss)
            
        # 更新按钮阴影效果
        for btn in self.findChildren(AnimatedButton):
            shadow = btn.graphicsEffect()
            if shadow:
                if self.theme_combo.currentIndex() == 2:  # 高对比度模式
                    shadow.setColor(QColor(255, 214, 0, 120))
                elif self.is_dark:  # 暗色模式
                    shadow.setColor(QColor(0, 0, 0, 100))
                else:  # 浅色模式
                    shadow.setColor(QColor(0, 0, 0, 80))

    def fade_theme(self, callback):
        # 动画期间禁用控件，防止多次切换
        self.theme_combo.setEnabled(False)
        self.btn_theme.setEnabled(False)
        fade_out = QPropertyAnimation(self.opacity_effect, b"opacity")
        fade_out.setDuration(220)
        fade_out.setStartValue(1.0)
        fade_out.setEndValue(0.0)
        fade_out.setEasingCurve(QEasingCurve.InOutQuad)
        fade_in = QPropertyAnimation(self.opacity_effect, b"opacity")
        fade_in.setDuration(220)
        fade_in.setStartValue(0.0)
        fade_in.setEndValue(1.0)
        fade_in.setEasingCurve(QEasingCurve.InOutQuad)
        def after_fade_out():
            callback()
            fade_in.start()
        def after_fade_in():
            self.theme_combo.setEnabled(True)
            self.btn_theme.setEnabled(True)
        fade_out.finished.connect(after_fade_out)
        fade_in.finished.connect(after_fade_in)
        fade_out.start()

    def run_slam(self):
        dialog = MapInputDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            filename = dialog.get_filename()
            if filename:
                try:
                    # 用subprocess启动explore.py，效果和命令行一致
                    subprocess.Popen(['python', 'explore2.py', filename])
                except Exception as e:
                    QMessageBox.warning(self, "错误", f"无法启动探索程序：{str(e)}")
            else:
                QMessageBox.warning(self, "提示", "请输入地图文件名！")

    def run_plan(self):# 弹窗淡入淡出可选实现，简单起见保持原样
        dialog2 = MapInputDialog2(self)
        if dialog2.exec_() == QDialog.Accepted:
            filename = dialog2.get_filename()
            if filename:
                try:
                    # 用subprocess启动explore.py，效果和命令行一致
                    subprocess.Popen(['python', 'map_and_path_planner.py', filename])
                except Exception as e:
                    QMessageBox.warning(self, "错误", f"无法启动路径规划：{str(e)}")
            else:
                QMessageBox.warning(self, "提示", "请输入地图数据文件名！")

    def run_draw_map(self):
        dialog = HandDrawnMapDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            # 如果用户已经保存了地图文件，那么dialog.get_data()会返回数据
            map_data = dialog.get_data()
            if map_data:
                try:
                    # 询问用户是否立即使用此地图运行探索程序
                    response = QMessageBox.question(
                        self, 
                        "运行探索程序", 
                        "是否立即使用此地图运行探索程序？",
                        QMessageBox.Yes | QMessageBox.No,
                        QMessageBox.Yes
                    )
                    
                    if response == QMessageBox.Yes:
                        # 从map_data获取文件名
                        filename = "hand_drawn_maze.json"  # 默认文件名
                        
                        # 运行探索程序
                        subprocess.Popen(['python', 'explore2.py', filename])
                except Exception as e:
                    QMessageBox.warning(self, "错误", f"无法启动探索程序：{str(e)}")

    def show_help(self):
        help_dialog = QMessageBox(self)
        help_dialog.setWindowTitle("帮助与支持")
        help_dialog.setText("<h2>雷达小车探索系统 - 帮助信息</h2>")
        help_dialog.setInformativeText("""
        <p style='font-size: 14px; line-height: 1.5;'>
        <b>SLAM扫描功能</b>: 使用激光雷达进行同步定位与地图构建，探索未知环境。<br><br>
        <b>路径规划功能</b>: 基于扫描数据，使用DWA或A*算法计算最优路径。<br><br>
        <b>手绘地图功能</b>: 手动绘制迷宫地图，并保存为JSON格式以用于探索和路径规划。<br><br>
        <b>操作提示</b>:<br>
        1. 确保输入文件存在且格式正确<br>
        2. 可以通过切换主题改变界面外观<br>
        3. 在手绘地图时，点击两点可创建墙壁，需要设置起点坐标<br>
        4. 如遇到系统错误，请重启应用<br><br>
        <span style='color: #666; font-style: italic;'>如有问题，请联系技术支持人员</span>
        </p>
        """)
        
        # 设置图标
        help_dialog.setIconPixmap(QPixmap("help_icon_large.png").scaledToHeight(80, Qt.SmoothTransformation))
        
        # 应用样式
        help_dialog.setStyleSheet("""
        QMessageBox {
            background-color: #f8faff;
            border-radius: 15px;
        }
        QLabel {
            color: #333;
            font-family: "微软雅黑";
        }
        QPushButton {
            background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #6ac8fe, stop:1 #4a90e2);
            color: white;
            border-radius: 10px;
            padding: 8px 16px;
            font-family: "微软雅黑";
            font-size: 14px;
            min-width: 100px;
        }
        QPushButton:hover {
            background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #7bd5ff, stop:1 #5a9ff2);
        }
        """)
        
        help_dialog.exec_()

    def add_floating_decorations(self):
        """添加悬浮卡通装饰元素"""
        # 创建云朵、星星等装饰元素
        decorations = []
        
        # 创建装饰性元素标签
        for i in range(5):  # 创建5个装饰元素
            label = QLabel(self)
            
            # 随机选择装饰类型
            deco_type = np.random.choice(["cloud", "star", "gear"])
            
            if deco_type == "cloud":
                if not self.cloud_icon.isNull():
                    label.setPixmap(self.cloud_icon.scaled(80, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                else:
                    # 创建一个简单的云朵
                    cloud = QPixmap(100, 60)
                    cloud.fill(Qt.transparent)
                    painter = QPainter(cloud)
                    painter.setRenderHint(QPainter.Antialiasing)
                    painter.setBrush(QBrush(QColor(255, 255, 255, 180)))
                    painter.setPen(QPen(QColor(200, 200, 255, 100), 1))
                    painter.drawEllipse(20, 15, 60, 40)
                    painter.drawEllipse(10, 25, 40, 30)
                    painter.drawEllipse(50, 20, 40, 35)
                    painter.end()
                    label.setPixmap(cloud)
            
            elif deco_type == "star":
                if not self.star_icon.isNull():
                    label.setPixmap(self.star_icon.scaled(40, 40, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                else:
                    # 创建一个简单的星星
                    star = QPixmap(40, 40)
                    star.fill(Qt.transparent)
                    painter = QPainter(star)
                    painter.setRenderHint(QPainter.Antialiasing)
                    painter.setBrush(QBrush(QColor(255, 255, 100, 200)))
                    painter.setPen(QPen(QColor(255, 200, 0, 150), 1))
                    
                    # 绘制五角星
                    points = []
                    center_x, center_y = 20, 20
                    outer_radius = 18
                    inner_radius = 8
                    for i in range(10):  # 5个外点，5个内点
                        angle = 2 * np.pi * i / 10 - np.pi / 2
                        radius = outer_radius if i % 2 == 0 else inner_radius
                        x = center_x + radius * np.cos(angle)
                        y = center_y + radius * np.sin(angle)
                        points.append(QPoint(int(x), int(y)))
                    
                    painter.drawPolygon(points)
                    painter.end()
                    label.setPixmap(star)
            
            else:  # gear
                if not self.gear_icon.isNull():
                    label.setPixmap(self.gear_icon.scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                else:
                    # 创建一个简单的齿轮
                    gear = QPixmap(50, 50)
                    gear.fill(Qt.transparent)
                    painter = QPainter(gear)
                    painter.setRenderHint(QPainter.Antialiasing)
                    painter.setBrush(QBrush(QColor(150, 150, 255, 180)))
                    painter.setPen(QPen(QColor(100, 100, 200, 150), 1))
                    
                    # 绘制齿轮外圈
                    center_x, center_y = 25, 25
                    outer_radius = 23
                    inner_radius = 18
                    teeth = 12
                    for i in range(teeth * 2):
                        angle = 2 * np.pi * i / (teeth * 2)
                        radius = outer_radius if i % 2 == 0 else inner_radius
                        x = center_x + radius * np.cos(angle)
                        y = center_y + radius * np.sin(angle)
                        if i == 0:
                            path = QPainterPath(QPoint(int(x), int(y)))
                        else:
                            path.lineTo(int(x), int(y))
                    path.closeSubpath()
                    painter.drawPath(path)
                    
                    # 绘制中心圆
                    painter.setBrush(QBrush(QColor(200, 200, 255, 180)))
                    painter.drawEllipse(center_x - 8, center_y - 8, 16, 16)
                    painter.end()
                    label.setPixmap(gear)
            
            # 设置初始位置在随机位置
            label.setGeometry(
                np.random.randint(50, self.width() - 100), 
                np.random.randint(50, self.height() - 100), 
                100, 60
            )
            
            # 创建动画
            self.create_floating_animation(label)
            
            label.show()
            decorations.append(label)
        
        self.decorations = decorations
    
    def create_floating_animation(self, widget):
        """为装饰元素创建浮动动画"""
        # 随机决定动画持续时间
        duration = np.random.randint(3000, 8000)
        
        # 创建位置动画
        animation = QPropertyAnimation(widget, b"pos")
        animation.setDuration(duration)
        animation.setStartValue(widget.pos())
        
        # 随机结束位置
        end_x = np.random.randint(50, self.width() - 100)
        end_y = np.random.randint(50, self.height() - 100)
        animation.setEndValue(QPoint(end_x, end_y))
        
        # 设置缓动曲线
        animation.setEasingCurve(QEasingCurve.InOutSine)
        
        # 当动画结束时，创建新的动画
        def on_finished():
            self.create_floating_animation(widget)
        
        animation.finished.connect(on_finished)
        animation.start()
        
        # 可选：添加一个透明度动画
        opacity_effect = QGraphicsOpacityEffect(widget)
        widget.setGraphicsEffect(opacity_effect)
        
        opacity_animation = QPropertyAnimation(opacity_effect, b"opacity")
        opacity_animation.setDuration(duration)
        opacity_animation.setStartValue(0.5 + np.random.random() * 0.5)  # 起始透明度
        opacity_animation.setEndValue(0.5 + np.random.random() * 0.5)    # 结束透明度
        opacity_animation.setEasingCurve(QEasingCurve.InOutSine)
        opacity_animation.start()

class MapInputDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("输入地图文件名")
        self.setFixedSize(1200, 500)
        
        # 应用主窗口的样式
        if parent and isinstance(parent, WelcomeWindow):
            if parent.is_dark:
                self.setStyleSheet(dark_qss)
            else:
                self.setStyleSheet(light_qss)
                
        # 设置圆角和阴影
        self.setStyleSheet(self.styleSheet() + """
            QDialog {
                border-radius: 20px;
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                                stop:0 #f0f6ff, stop:1 #e6f0ff);
            }
            QLineEdit {
                border-radius: 10px;
                padding: 10px;
                font-size: 18px;
                border: 2px solid #4a90e2;
                background-color: rgba(255, 255, 255, 0.8);
            }
        """)
        
        layout = QVBoxLayout()
        layout.setContentsMargins(40, 40, 40, 40)
        layout.setSpacing(30)
        
        # 标题
        title = QLabel("地图文件选择")
        title_font = QFont("微软雅黑", 24, QFont.Bold)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #4a90e2; margin-bottom: 5px;")
        layout.addWidget(title)
        
        # 说明标签
        label = QLabel("请输入要探索的地图文件（如 maze.json）：")
        label_font = QFont("微软雅黑", 16)
        label.setFont(label_font)
        label.setStyleSheet("margin-bottom: 15px;")
        layout.addWidget(label)
        
        # 输入框和按钮的容器
        input_frame = QFrame()
        input_frame.setStyleSheet("""
            QFrame {
                border-radius: 10px;
                background-color: rgba(255, 255, 255, 0.5);
                padding: 5px;
            }
        """)
        
        hbox = QHBoxLayout(input_frame)
        hbox.setContentsMargins(10, 10, 10, 10)
        hbox.setSpacing(10)
        
        # 输入框
        self.edit = QLineEdit()
        edit_font = QFont("微软雅黑", 18)
        self.edit.setFont(edit_font)
        self.edit.setPlaceholderText("如 maze.json")
        self.edit.setMinimumHeight(60)
        hbox.addWidget(self.edit, 3) # 分配更多空间给输入框
        
        # 提交按钮
        self.btn_submit = QPushButton("提交")
        self.btn_submit.setMinimumHeight(60)
        self.btn_submit.setFixedWidth(150)
        submit_font = QFont("微软雅黑", 18, QFont.Bold)
        self.btn_submit.setFont(submit_font)
        
        # 添加阴影效果
        shadow = QGraphicsDropShadowEffect(self.btn_submit)
        shadow.setBlurRadius(15)
        shadow.setColor(QColor(0, 0, 0, 80))
        shadow.setOffset(0, 4)
        self.btn_submit.setGraphicsEffect(shadow)
        
        hbox.addWidget(self.btn_submit)
        
        layout.addWidget(input_frame)
        
        # 添加一个提示信息
        tip_label = QLabel("提示: 确保文件在正确的目录位置")
        tip_font = QFont("微软雅黑", 14)
        tip_label.setFont(tip_font)
        tip_label.setStyleSheet("color: #888; font-style: italic; margin-top: 20px;")
        tip_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(tip_label)
        
        self.setLayout(layout)
        self.btn_submit.clicked.connect(self.accept)

    def get_filename(self):
        return self.edit.text().strip()

class MapInputDialog2(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("输入地图数据文件名")
        self.setFixedSize(1200, 500)
        
        # 应用主窗口的样式
        if parent and isinstance(parent, WelcomeWindow):
            if parent.is_dark:
                self.setStyleSheet(dark_qss)
            else:
                self.setStyleSheet(light_qss)
                
        # 设置圆角和阴影
        self.setStyleSheet(self.styleSheet() + """
            QDialog {
                border-radius: 20px;
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                                stop:0 #e8f0ff, stop:1 #dce8ff);
            }
            QLineEdit {
                border-radius: 10px;
                padding: 10px;
                font-size: 18px;
                border: 2px solid #364678;
                background-color: rgba(255, 255, 255, 0.8);
            }
        """)
        
        layout = QVBoxLayout()
        layout.setContentsMargins(40, 40, 40, 40)
        layout.setSpacing(30)
        
        # 标题
        title = QLabel("路径规划数据文件")
        title_font = QFont("微软雅黑", 24, QFont.Bold)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #364678; margin-bottom: 5px;")
        layout.addWidget(title)
        
        # 说明标签
        label = QLabel("请输入规划路径所需的地图数据文件：")
        label_font = QFont("微软雅黑", 16)
        label.setFont(label_font)
        label.setStyleSheet("margin-bottom: 15px;")
        layout.addWidget(label)
        
        # 输入框和按钮的容器
        input_frame = QFrame()
        input_frame.setStyleSheet("""
            QFrame {
                border-radius: 10px;
                background-color: rgba(255, 255, 255, 0.5);
                padding: 5px;
            }
        """)
        
        hbox = QHBoxLayout(input_frame)
        hbox.setContentsMargins(10, 10, 10, 10)
        hbox.setSpacing(10)
        
        # 输入框
        self.edit = QLineEdit()
        edit_font = QFont("微软雅黑", 18)
        self.edit.setFont(edit_font)
        self.edit.setPlaceholderText("如 maze_radar_data.txt")
        self.edit.setMinimumHeight(60)
        hbox.addWidget(self.edit, 3)
        
        # 提交按钮
        self.btn_submit = QPushButton("开始规划")
        self.btn_submit.setMinimumHeight(60)
        self.btn_submit.setFixedWidth(180)
        submit_font = QFont("微软雅黑", 18, QFont.Bold)
        self.btn_submit.setFont(submit_font)
        
        # 设置路径规划按钮的样式
        self.btn_submit.setStyleSheet("""
            QPushButton {
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, 
                                                 stop:0 #4f6294, stop:1 #364678);
                color: white;
                border-radius: 15px;
            }
            QPushButton:hover {
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, 
                                                 stop:0 #5f72a4, stop:1 #465688);
            }
        """)
        
        # 添加阴影效果
        shadow = QGraphicsDropShadowEffect(self.btn_submit)
        shadow.setBlurRadius(15)
        shadow.setColor(QColor(0, 0, 0, 80))
        shadow.setOffset(0, 4)
        self.btn_submit.setGraphicsEffect(shadow)
        
        hbox.addWidget(self.btn_submit)
        
        layout.addWidget(input_frame)
        
        # 添加一个提示信息
        tip_label = QLabel("提示: 将使用A和A*算法分别规划，并使用DWA实现小车轨迹")
        tip_font = QFont("微软雅黑", 14)
        tip_label.setFont(tip_font)
        tip_label.setStyleSheet("color: #888; font-style: italic; margin-top: 20px;")
        tip_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(tip_label)
        
        self.setLayout(layout)
        self.btn_submit.clicked.connect(self.accept)

    def get_filename(self):
        return self.edit.text().strip()

class HandDrawnMapDialog(QDialog):
    """手绘地图对话框"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("手绘迷宫地图")
        
        # 初始化数据
        self.grid_size = 15
        self.cell_size = 70  # 保持网格尺寸
        self.margin = 30     # 保持边距
        
        # 计算窗口大小
        window_width = self.grid_size * self.cell_size + 4 * self.margin
        window_height = self.grid_size * self.cell_size + 4 * self.margin + 220  # 减小额外空间，因为合并了坐标输入和按钮
        
        self.setFixedSize(max(800, window_width), max(800, window_height))
        self.setModal(True)
        self.walls = []  # 存储墙壁线段
        self.selected_points = []  # 存储当前选中的点
        self.start_point = None
        
        # 设置样式
        self.setStyleSheet("""
            QDialog {
                background-color: #f8f9fa;
            }
            QLabel {
                font-size: 20px;  /* 从16px增加到20px */
                color: #212529;
            }
            QPushButton {
                font-size: 20px;  /* 从16px增加到20px */
                padding: 16px 30px;  /* 从12px 25px增加到16px 30px */
                border-radius: 12px;  /* 从8px增加到12px */
                background-color: #4361ee;
                color: white;
                border: none;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #3046eb;
            }
            QLineEdit {
                font-size: 20px;  /* 从16px增加到20px */
                padding: 12px;  /* 从8px增加到12px */
                border: 3px solid #4361ee;  /* 从2px增加到3px */
                border-radius: 10px;  /* 从6px增加到10px */
                background-color: white;
            }
            QFrame#grid_container {
                background-color: white;
                border: 3px solid #dee2e6;  /* 从2px增加到3px */
                border-radius: 15px;  /* 从10px增加到15px */
            }
        """)
        
        self._init_ui()
    
    def _init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        layout.setSpacing(20)  # 保持间距
        layout.setContentsMargins(40, 30, 40, 40)  # 增加底部边距
        
        # 标题
        title = QLabel("✏️ 手绘迷宫地图")
        title.setStyleSheet("font-size: 32px; font-weight: bold; color: #4361ee;")  # 从24px增加到32px
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # 说明文字
        description = QLabel(
            "• 点击两个点创建水平或竖直的墙壁（可以任意长度）\n"
            "• 再次点击相同的两个点删除墙壁\n"
            "• 输入起点坐标 (0-14)\n"
            "• 点击确认生成地图"
        )
        description.setStyleSheet("""
            font-size: 18px;  /* 减小字体 */
            color: #495057;
            padding: 15px;  /* 减小内边距 */
            background-color: #e9ecef;
            border-radius: 12px;
        """)
        description.setAlignment(Qt.AlignLeft)
        layout.addWidget(description)

        # 添加一个占位空间，用于网格绘制区域
        spacer = QWidget()
        spacer.setFixedSize(
            self.grid_size * self.cell_size + 2 * self.margin,
            self.grid_size * self.cell_size + 2 * self.margin
        )
        spacer.setStyleSheet("background: transparent;")
        layout.addWidget(spacer, alignment=Qt.AlignCenter)

        # 添加额外的垂直空间
        layout.addSpacing(20)
        
        # 创建底部区域（包含起点坐标输入和按钮）
        bottom_layout = QHBoxLayout()
        bottom_layout.setSpacing(15)
        
        # 清除按钮 (放在左侧)
        self.clear_button = QPushButton("🗑️ 清除")
        self.clear_button.setMinimumHeight(50)
        self.clear_button.setMinimumWidth(120)
        self.clear_button.clicked.connect(self._clear_map)
        bottom_layout.addWidget(self.clear_button)
        
        # 添加一些空间
        bottom_layout.addSpacing(20)
        
        # 起点坐标标签
        start_label = QLabel("起点坐标:")
        start_label.setFont(QFont("微软雅黑", 16))
        bottom_layout.addWidget(start_label)
        
        # X坐标输入框
        self.start_x = QLineEdit()
        self.start_x.setPlaceholderText("X (0-14)")
        self.start_x.setFixedWidth(100)
        self.start_x.setFixedHeight(40)
        bottom_layout.addWidget(self.start_x)
        
        # 逗号
        comma_label = QLabel("，")
        comma_label.setFont(QFont("微软雅黑", 16))
        bottom_layout.addWidget(comma_label)
        
        # Y坐标输入框
        self.start_y = QLineEdit()
        self.start_y.setPlaceholderText("Y (0-14)")
        self.start_y.setFixedWidth(100)
        self.start_y.setFixedHeight(40)
        bottom_layout.addWidget(self.start_y)

        # 右侧弹性空间
        bottom_layout.addStretch(1)
        
        # 确认和取消按钮
        self.confirm_button = QPushButton("✅ 确认")
        self.confirm_button.setMinimumHeight(50)
        self.confirm_button.setMinimumWidth(120)
        self.confirm_button.clicked.connect(self._confirm_map)
        bottom_layout.addWidget(self.confirm_button)
        
        self.cancel_button = QPushButton("❌ 取消")
        self.cancel_button.setMinimumHeight(50)
        self.cancel_button.setMinimumWidth(120)
        self.cancel_button.clicked.connect(self.reject)
        bottom_layout.addWidget(self.cancel_button)
        
        layout.addLayout(bottom_layout)
    
    def paintEvent(self, event):
        """绘制网格和墙壁"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 计算网格区域
        window_width = self.width()
        window_height = self.height()
        grid_width = self.grid_size * self.cell_size
        grid_height = self.grid_size * self.cell_size
        
        # 计算居中位置，确保顶部有足够空间
        x_offset = (window_width - grid_width) // 2
        y_offset = ((window_height - grid_height) // 2) + 40  # 增加顶部空间
        
        grid_rect = QRect(
            x_offset,
            y_offset,
            grid_width,
            grid_height
        )
        
        # 绘制网格
        painter.setPen(QPen(QColor("#dee2e6"), 2))  # 从1增加到2
        for i in range(self.grid_size + 1):
            # 垂直线
            x = grid_rect.left() + i * self.cell_size
            painter.drawLine(x, grid_rect.top(), x, grid_rect.bottom())
            # 水平线
            y = grid_rect.bottom() - i * self.cell_size  # 从底部开始画
            painter.drawLine(grid_rect.left(), y, grid_rect.right(), y)
        
        # 绘制网格点
        painter.setPen(QPen(QColor("#4361ee"), 12))  # 从6增加到12
        for i in range(self.grid_size + 1):
            for j in range(self.grid_size + 1):
                x = grid_rect.left() + i * self.cell_size
                y = grid_rect.bottom() - j * self.cell_size  # 从底部开始计算
                painter.drawPoint(x, y)
        
        # 绘制已选择的点
        if self.selected_points:
            painter.setPen(QPen(QColor("#28a745"), 16))  # 从8增加到16
            for point in self.selected_points:
                x = grid_rect.left() + point[0] * self.cell_size
                y = grid_rect.bottom() - point[1] * self.cell_size  # 从底部开始计算
                painter.drawPoint(x, y)
        
        # 绘制墙壁
        painter.setPen(QPen(QColor("#212529"), 6))  # 从3增加到6
        for wall in self.walls:
            start_x = grid_rect.left() + wall[0][0] * self.cell_size
            start_y = grid_rect.bottom() - wall[0][1] * self.cell_size  # 从底部开始计算
            end_x = grid_rect.left() + wall[1][0] * self.cell_size
            end_y = grid_rect.bottom() - wall[1][1] * self.cell_size  # 从底部开始计算
            painter.drawLine(start_x, start_y, end_x, end_y)
        
        # 绘制起点
        if self.start_point is not None:
            painter.setPen(QPen(QColor("#dc3545"), 8))  # 从4增加到8
            painter.setBrush(QColor("#dc3545"))
            x = grid_rect.left() + self.start_point[0] * self.cell_size
            y = grid_rect.bottom() - self.start_point[1] * self.cell_size  # 从底部开始计算
            painter.drawEllipse(QPoint(x, y), 16, 16)  # 从8,8增加到16,16
    
    def mousePressEvent(self, event):
        """处理鼠标点击事件"""
        # 计算网格区域
        window_width = self.width()
        window_height = self.height()
        grid_width = self.grid_size * self.cell_size
        grid_height = self.grid_size * self.cell_size
        
        # 计算居中位置，确保顶部有足够空间
        x_offset = (window_width - grid_width) // 2
        y_offset = ((window_height - grid_height) // 2) + 40
        
        grid_rect = QRect(
            x_offset,
            y_offset,
            grid_width,
            grid_height
        )
        
        # 检查点击是否在网格区域内
        if not grid_rect.contains(event.pos()):
            return
        
        # 计算点击的网格坐标
        x = round((event.pos().x() - grid_rect.left()) / self.cell_size)
        y = self.grid_size - round((event.pos().y() - grid_rect.top()) / self.cell_size)  # 从底部开始计算
        
        # 确保坐标在有效范围内
        if 0 <= x <= self.grid_size and 0 <= y <= self.grid_size:
            point = [x, y]
            
            # 如果已经选择了一个点
            if self.selected_points:
                # 检查是否与第一个点相同
                if point == self.selected_points[0]:
                    self.selected_points.clear()
                else:
                    # 添加第二个点并创建或删除墙
                    self.selected_points.append(point)
                    self._handle_wall()
                    self.selected_points.clear()
            else:
                # 选择第一个点
                self.selected_points.append(point)
            
            # 更新显示
            self.update()
    
    def _handle_wall(self):
        """处理墙壁的添加或删除"""
        if len(self.selected_points) != 2:
            return
            
        p1, p2 = self.selected_points
        
        # 检查是否是水平或竖直的墙
        if p1[0] != p2[0] and p1[1] != p2[1]:
            # 如果既不是水平也不是竖直，则不创建墙
            return
            
        # 确保p1的坐标小于p2的坐标
        if p1[0] > p2[0] or (p1[0] == p2[0] and p1[1] > p2[1]):
            p1, p2 = p2, p1
            
        # 检查是否已存在这面墙
        wall = (tuple(p1), tuple(p2))
        
        # 查找是否已存在完全相同的墙
        for existing_wall in self.walls:
            if (tuple(existing_wall[0]) == wall[0] and tuple(existing_wall[1]) == wall[1]) or \
               (tuple(existing_wall[0]) == wall[1] and tuple(existing_wall[1]) == wall[0]):
                # 删除已存在的墙
                self.walls.remove(existing_wall)
                return
        
        # 添加新墙
        self.walls.append([list(wall[0]), list(wall[1])])
    
    def _clear_map(self):
        """清除地图"""
        self.walls.clear()
        self.selected_points.clear()
        self.start_point = None
        self.start_x.clear()
        self.start_y.clear()
        self.update()
    
    def _confirm_map(self):
        """确认并生成地图"""
        # 验证起点坐标
        try:
            x = int(self.start_x.text())
            y = int(self.start_y.text())
            if not (0 <= x <= 14 and 0 <= y <= 14):
                raise ValueError()
            self.start_point = [x, y]
        except (ValueError, TypeError):
            QMessageBox.warning(self, "错误", "请输入有效的起点坐标 (0-14)！")
            return
        
        # 验证是否有墙
        if not self.walls:
            QMessageBox.warning(self, "错误", "请至少创建一面墙！")
            return
        
        # 生成JSON数据
        self.data = {
            "segments": [
                {
                    "start": wall[0],
                    "end": wall[1]
                }
                for wall in self.walls
            ],
            "start_point": self.start_point
        }
        
        # 保存JSON文件
        file_name, _ = QFileDialog.getSaveFileName(self, "保存地图", "", "JSON文件 (*.json)")
        if not file_name:
            return
            
        if not file_name.endswith(".json"):
            file_name += ".json"
            
        with open(file_name, "w") as f:
            json.dump(self.data, f, indent=2)
        
        QMessageBox.information(self, "成功", f"地图已保存至 {file_name}！")
        self.accept()
        
    def get_data(self):
        """返回地图数据"""
        if hasattr(self, 'data'):
            return self.data
        return None

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WelcomeWindow()
    window.show()
    sys.exit(app.exec_())