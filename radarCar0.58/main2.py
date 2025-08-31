from maze_utils import load_edges, draw_maze
from robot_utils import Robot, simulate_lidar_scan
from explore2 import explore_maze  # 新增：导入探索主函数
import matplotlib.pyplot as plt
import numpy as np
import sys
import subprocess
from PyQt5.QtCore import Qt, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton, QVBoxLayout, QLabel, QMessageBox, QHBoxLayout, QSizePolicy
from PyQt5.QtWidgets import QGraphicsOpacityEffect, QComboBox, QDialog, QLineEdit

# ===== 主题配色方案 =====
light_qss = """
QWidget {
    background-color: #f5f6fa;
    color: #222;
}
QPushButton {
    background-color: #4a90e2;
    color: #fff;
    border-radius: 8px;
    font-size: 18px;
    padding: 8px 16px;
}
QPushButton:hover {
    background-color: #357ab8;
}
QLabel {
    color: #222;
}
"""
dark_qss = """
QWidget {
    background-color: #23272e;
    color: #e6e6e6;
}
QPushButton {
    background-color: #3a6ea5;
    color: #fff;
    border-radius: 8px;
    font-size: 18px;
    padding: 8px 16px;
}
QPushButton:hover {
    background-color: #27496d;
}
QLabel {
    color: #e6e6e6;
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

    def mousePressEvent(self, event):
        self._orig_geometry = self.geometry()
        shrink = self._orig_geometry.adjusted(4, 4, -4, -4)
        self._animation.stop()
        self._animation.setStartValue(self._orig_geometry)
        self._animation.setEndValue(shrink)
        self._animation.start()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if self._orig_geometry:
            self._animation.stop()
            self._animation.setStartValue(self.geometry())
            self._animation.setEndValue(self._orig_geometry)
            self._animation.start()
        super().mouseReleaseEvent(event)

class WelcomeWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.is_dark = False
        self.setWindowTitle("雷达小车探索系统")
        self.setGeometry(400, 200, 400, 300)
        self.opacity_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self.opacity_effect)
        self.init_ui()
        self.apply_theme()

    def init_ui(self):
        central_widget = QWidget()
        self.main_layout = QVBoxLayout()
        self.main_layout.setContentsMargins(40, 32, 40, 32)
        self.main_layout.setSpacing(28)

        self.label_stretch_top = self.main_layout.addStretch(2)

        # ===== 顶部LOGO图片 =====
        logo_label = QLabel()
        logo_pix = QPixmap("logo.png")
        if logo_pix.isNull():
            logo_label.setText("[LOGO]")
            logo_label.setStyleSheet("font-size: 28px; color: #888;")
        else:
            logo_label.setPixmap(logo_pix.scaledToHeight(80, Qt.SmoothTransformation))
        logo_label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(logo_label)

        label = QLabel("欢迎使用雷达小车探索系统")
        label.setStyleSheet("font-size: 22px; font-weight: bold; margin-bottom: 18px;")
        label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(label)

        self.btn_layout = QHBoxLayout()
        self.btn_layout.setSpacing(24)
        self.btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_slam = AnimatedButton("SLAM扫描")
        btn_plan = AnimatedButton("路径规划（DWA/A*）")
        # ===== 按钮加图标 =====
        btn_slam.setIcon(QIcon("radar_icon.png"))
        btn_plan.setIcon(QIcon("plan_icon.png"))
        for btn in (btn_slam, btn_plan):
            btn.setMinimumHeight(48)
            btn.setMaximumHeight(48)
            btn.setMinimumWidth(120)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_layout.addStretch(1)
        self.btn_layout.addWidget(btn_slam)
        self.btn_layout.addWidget(btn_plan)
        self.btn_layout.addStretch(1)
        self.main_layout.addLayout(self.btn_layout)

        self.theme_layout = QHBoxLayout()
        self.theme_layout.setSpacing(12)
        self.theme_layout.setContentsMargins(0, 0, 0, 0)
        theme_label = QLabel("主题模式：")
        theme_label.setStyleSheet("font-size: 15px;")
        self.theme_combo = QComboBox()
        self.theme_combo.addItems(["浅色模式", "暗色模式", "高对比度模式"])
        self.theme_combo.setCurrentIndex(1 if self.is_dark else 0)
        self.theme_combo.setFixedHeight(36)
        self.theme_combo.setMinimumWidth(100)
        self.theme_combo.currentIndexChanged.connect(self.on_theme_selected)
        self.btn_theme = AnimatedButton("切换主题")
        self.btn_theme.setFixedHeight(36)
        self.btn_theme.setMinimumWidth(90)
        self.btn_theme.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        # 主题按钮加图标
        self.btn_theme.setIcon(QIcon("theme_icon.png"))
        self.btn_theme.clicked.connect(self.on_theme_button_clicked)
        self.theme_layout.addStretch(1)
        self.theme_layout.addWidget(theme_label, alignment=Qt.AlignVCenter)
        self.theme_layout.addWidget(self.theme_combo, alignment=Qt.AlignVCenter)
        self.theme_layout.addWidget(self.btn_theme, alignment=Qt.AlignVCenter)
        self.theme_layout.addStretch(1)
        self.main_layout.addLayout(self.theme_layout)

        # 增加帮助按钮
        btn_help = AnimatedButton("帮助")
        btn_help.setAccessibleName("帮助按钮")
        btn_help.setAccessibleDescription("点击以获取帮助信息")
        btn_help.setFocusPolicy(Qt.StrongFocus)
        btn_help.setFixedHeight(36)
        btn_help.setMinimumWidth(90)
        btn_help.setStyleSheet("font-size: 18px;")
        btn_help.clicked.connect(self.show_help)
        self.main_layout.addWidget(btn_help, alignment=Qt.AlignRight)

        self.label_stretch_bottom = self.main_layout.addStretch(3)

        central_widget.setLayout(self.main_layout)
        self.setCentralWidget(central_widget)

        # ===== 设置简洁渐变背景 =====
        self.setStyleSheet(self.styleSheet() + "\nQWidget#WelcomeWindow {background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #e9f1fb, stop:1 #f5f6fa);}")
        self.setObjectName("WelcomeWindow")

        btn_slam.clicked.connect(self.run_slam)
        btn_plan.clicked.connect(self.run_plan)

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
            QPushButton { background: #FFD600; color: #000; font-weight: bold; border-radius: 8px; font-size: 18px; }
            QPushButton:hover { background: #FFF176; color: #000; }
            QLabel { color: #FFD600; }
            QComboBox { background: #222; color: #FFD600; border-radius: 6px; font-size: 16px; }
            QComboBox QAbstractItemView { background: #222; color: #FFD600; }
            QMessageBox { background: #000; color: #FFD600; }
            """)
        elif self.is_dark:
            self.setStyleSheet(dark_qss)
        else:
            self.setStyleSheet(light_qss)

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

    def show_help(self):
        QMessageBox.information(self, "帮助", "出现错误，建议停止，并询问有关人员")

class MapInputDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("输入地图文件名")
        self.setFixedSize(340, 120)
        layout = QVBoxLayout()
        label = QLabel("请输入要探索的地图文件（如 maze.json）：")
        self.edit = QLineEdit()
        self.edit.setPlaceholderText("如 maze.json")
        hbox = QHBoxLayout()
        hbox.addWidget(self.edit)
        self.btn_submit = QPushButton("提交")
        hbox.addWidget(self.btn_submit)
        layout.addWidget(label)
        layout.addLayout(hbox)
        self.setLayout(layout)
        self.btn_submit.clicked.connect(self.accept)

    def get_filename(self):
        return self.edit.text().strip()

class MapInputDialog2(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("输入地图数据文件名")
        self.setFixedSize(340, 120)
        layout = QVBoxLayout()
        label = QLabel("请输入规划路径所需要的地图数据文件（如 maze_radar_data.txt）：")
        self.edit = QLineEdit()
        self.edit.setPlaceholderText("如maze_radar_data.txt")
        hbox = QHBoxLayout()
        hbox.addWidget(self.edit)
        self.btn_submit = QPushButton("提交")
        hbox.addWidget(self.btn_submit)
        layout.addWidget(label)
        layout.addLayout(hbox)
        self.setLayout(layout)
        self.btn_submit.clicked.connect(self.accept)

    def get_filename(self):
        return self.edit.text().strip()
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WelcomeWindow()
    window.show()
    sys.exit(app.exec_())

    # 1. 加载迷宫边缘线段
    edges = load_edges('maze_edges.json')

    # 2. 创建一个 matplotlib 坐标系
    fig, ax = plt.subplots(figsize=(10, 10))

    # 3. 在坐标系上绘制迷宫
    draw_maze(edges, ax=ax)

    # 4. 创建机器人实例并设置起点和初始朝向
    #    theta=0 表示朝向x轴正方向 (右)
    #    theta=pi/2 表示朝向y轴正方向 (上)
    start_x, start_y = 5, 5
    start_theta = np.deg2rad(0) # 初始朝向45度
    robot = Robot(x=start_x, y=start_y, theta=start_theta)

    # 5. 模拟一次激光雷达扫描
    scan_data = simulate_lidar_scan(robot, edges)

    # 6. 在控制台打印雷达数据，方便查看
    print(f"--- 雷达扫描数据 ---")
    print(f"扫描完成，共得到 {len(scan_data)} 个读数。")
    print("前 5 个读数 (角度, 距离):")
    for i in range(min(5, len(scan_data))): # 使用min确保列表长度足够
        angle_rad, dist = scan_data[i]
        angle_deg = np.rad2deg(angle_rad) % 360 # 转为0-360度的角
        print(f"  - 机器人相对角度: {i * 360.0/len(scan_data):.1f} 度,  距离: {dist:.2f} 米")
    print("--------------------")

    # 7. 在同一张图上绘制机器人和雷达扫描
    draw_robot_and_scan(ax, robot, scan_data)
    # 8. 添加图例并显示最终图像
    ax.legend()
    plt.show()