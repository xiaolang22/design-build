#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手绘迷宫地图对话框
"""
import json
from pathlib import Path
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFrame, QLineEdit, QMessageBox, QWidget
)
from PySide6.QtCore import Qt, QPoint, QRect
from PySide6.QtGui import QPainter, QPen, QColor

class DrawMapDialog(QDialog):
    """手绘地图对话框"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("手绘迷宫地图")
        
        # 初始化数据
        self.grid_size = 15
        self.cell_size = 40
        self.margin = 20
        
        # 计算窗口大小
        window_width = self.grid_size * self.cell_size + 4 * self.margin
        window_height = self.grid_size * self.cell_size + 6 * self.margin + 250  # 额外空间用于其他UI元素
        
        self.setFixedSize(max(800, window_width), max(900, window_height))
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
                font-size: 16px;
                color: #212529;
            }
            QPushButton {
                font-size: 16px;
                padding: 12px 25px;
                border-radius: 8px;
                background-color: #4361ee;
                color: white;
                border: none;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #3046eb;
            }
            QLineEdit {
                font-size: 16px;
                padding: 8px;
                border: 2px solid #4361ee;
                border-radius: 6px;
                background-color: white;
            }
            QFrame#grid_container {
                background-color: white;
                border: 2px solid #dee2e6;
                border-radius: 10px;
            }
        """)
        
        self._init_ui()
    
    def _init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        layout.setSpacing(20)
        layout.setContentsMargins(30, 30, 30, 30)
        
        # 标题
        title = QLabel("✏️ 手绘迷宫地图")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #4361ee;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)
        
        # 说明文字
        description = QLabel(
            "• 点击两个点创建水平或竖直的墙壁（可以任意长度）\n"
            "• 再次点击相同的两个点删除墙壁\n"
            "• 输入起点坐标 (0-14)\n"
            "• 点击确认生成地图"
        )
        description.setStyleSheet("""
            font-size: 16px;
            color: #495057;
            padding: 15px;
            background-color: #e9ecef;
            border-radius: 8px;
        """)
        description.setAlignment(Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(description)
        
        # 添加一个占位空间，用于网格绘制区域
        spacer = QWidget()
        spacer.setFixedSize(
            self.grid_size * self.cell_size + 2 * self.margin,
            self.grid_size * self.cell_size + 2 * self.margin
        )
        spacer.setStyleSheet("background: transparent;")
        layout.addWidget(spacer, alignment=Qt.AlignmentFlag.AlignCenter)
        
        # 起点坐标输入
        coord_layout = QHBoxLayout()
        coord_layout.setSpacing(10)
        
        start_label = QLabel("起点坐标:")
        coord_layout.addWidget(start_label)
        
        self.start_x = QLineEdit()
        self.start_x.setPlaceholderText("X (0-14)")
        self.start_x.setFixedWidth(100)
        coord_layout.addWidget(self.start_x)
        
        coord_layout.addWidget(QLabel("，"))
        
        self.start_y = QLineEdit()
        self.start_y.setPlaceholderText("Y (0-14)")
        self.start_y.setFixedWidth(100)
        coord_layout.addWidget(self.start_y)
        
        coord_layout.addStretch()
        layout.addLayout(coord_layout)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        button_layout.setSpacing(15)
        
        self.clear_button = QPushButton("🗑️ 清除")
        self.clear_button.clicked.connect(self._clear_map)
        button_layout.addWidget(self.clear_button)
        
        button_layout.addStretch()
        
        self.confirm_button = QPushButton("✅ 确认")
        self.confirm_button.clicked.connect(self._confirm_map)
        button_layout.addWidget(self.confirm_button)
        
        self.cancel_button = QPushButton("❌ 取消")
        self.cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(self.cancel_button)
        
        layout.addLayout(button_layout)
    
    def paintEvent(self, event):
        """绘制网格和墙壁"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
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
        painter.setPen(QPen(QColor("#dee2e6"), 1))
        for i in range(self.grid_size + 1):
            # 垂直线
            x = grid_rect.left() + i * self.cell_size
            painter.drawLine(x, grid_rect.top(), x, grid_rect.bottom())
            # 水平线
            y = grid_rect.bottom() - i * self.cell_size  # 从底部开始画
            painter.drawLine(grid_rect.left(), y, grid_rect.right(), y)
        
        # 绘制网格点
        painter.setPen(QPen(QColor("#4361ee"), 6))
        for i in range(self.grid_size + 1):
            for j in range(self.grid_size + 1):
                x = grid_rect.left() + i * self.cell_size
                y = grid_rect.bottom() - j * self.cell_size  # 从底部开始计算
                painter.drawPoint(x, y)
        
        # 绘制已选择的点
        if self.selected_points:
            painter.setPen(QPen(QColor("#28a745"), 8))
            for point in self.selected_points:
                x = grid_rect.left() + point[0] * self.cell_size
                y = grid_rect.bottom() - point[1] * self.cell_size  # 从底部开始计算
                painter.drawPoint(x, y)
        
        # 绘制墙壁
        painter.setPen(QPen(QColor("#212529"), 3))
        for wall in self.walls:
            start_x = grid_rect.left() + wall[0][0] * self.cell_size
            start_y = grid_rect.bottom() - wall[0][1] * self.cell_size  # 从底部开始计算
            end_x = grid_rect.left() + wall[1][0] * self.cell_size
            end_y = grid_rect.bottom() - wall[1][1] * self.cell_size  # 从底部开始计算
            painter.drawLine(start_x, start_y, end_x, end_y)
        
        # 绘制起点
        if self.start_point is not None:
            painter.setPen(QPen(QColor("#dc3545"), 4))
            painter.setBrush(QColor("#dc3545"))
            x = grid_rect.left() + self.start_point[0] * self.cell_size
            y = grid_rect.bottom() - self.start_point[1] * self.cell_size  # 从底部开始计算
            painter.drawEllipse(QPoint(x, y), 8, 8)
    
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
        data = {
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
        file_path = Path(__file__).resolve().parent / "hand_drawn_maze.json"
        with open(file_path, "w") as f:
            json.dump(data, f, indent=2)
        
        QMessageBox.information(self, "成功", "地图已保存！")
        self.accept() 