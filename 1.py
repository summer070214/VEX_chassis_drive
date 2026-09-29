"""
VEX V5 专业上位机调试软件 v1.8
修复：Y坐标映射逻辑 + 角度定义（Y正方向为0度）
全场定位地图功能 (6x6场地, 每格60cm)
坐标系统：左下角为原点，X向右为正，Y向上为正
角度定义：Y轴正方向为0度，范围-180~180
"""

import sys
import serial
import serial.tools.list_ports
from collections import deque
import threading
import time
import csv
from datetime import datetime

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import pyqtgraph as pg
import math

# 配置
MAX_POINTS = 1000
FIELD_SIZE = 6  # 6x6 场地
CELL_SIZE = 80  # 每个格子像素大小
CELL_CM = 60    # 每个格子实际长度 60cm

class SerialWorker(QThread):
    """串口数据读取线程"""
    data_received = pyqtSignal(dict)
    status_changed = pyqtSignal(str)
    position_received = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.serial = None
        self.running = False
        self.port = None
        self.baudrate = 115200
        
    def connect(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            self.running = True
            self.status_changed.emit(f"已连接 {port}")
            return True
        except Exception as e:
            self.status_changed.emit(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.status_changed.emit("已断开")
    
    def run(self):
        while self.running and self.serial:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    
                    # 处理位置更新命令 (VEX发送的坐标是cm单位，需要转换为格子坐标)
                    if line.startswith('POS_UPDATE:'):
                        parts = line[11:].split(',')
                        if len(parts) >= 3:
                            try:
                                # VEX发送的是实际cm坐标，需要转换为格子坐标 (0~6)
                                x_cm = float(parts[0])
                                y_cm = float(parts[1])
                                angle = float(parts[2])
                                
                                # cm转格子坐标 (每个格子60cm)
                                x_grid = x_cm / CELL_CM
                                y_grid = y_cm / CELL_CM
                                
                                pos_data = {
                                    'x': x_grid,
                                    'y': y_grid,
                                    'angle': angle
                                }
                                self.position_received.emit(pos_data)
                            except:
                                pass
                    
                    # 处理数据曲线命令
                    elif line.startswith('DATA:'):
                        parts = line[5:].split(',')
                        if len(parts) >= 5:
                            try:
                                data = {
                                    'time': float(parts[0]),
                                    'ch1': float(parts[1]),
                                    'ch2': float(parts[2]),
                                    'ch3': float(parts[3]),
                                    'ch4': float(parts[4])
                                }
                                self.data_received.emit(data)
                            except:
                                pass
                else:
                    self.msleep(1)
            except:
                break
    
    def send_command(self, cmd):
        if self.serial and self.serial.is_open:
            self.serial.write((cmd + '\n').encode())

class FieldMapWidget(QWidget):
    """场地地图显示组件 - 左下角原点，Y向上为正，X向右为正"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.robot_x = 3.0   # 格子坐标 X (0-6)
        self.robot_y = 3.0   # 格子坐标 Y (0-6)
        self.robot_angle = 0.0
        self.setMinimumSize(500, 500)
        
    def update_robot_position(self, x, y, angle):
        """更新机器人位置 (格子坐标 0~6)"""
        self.robot_x = max(0, min(FIELD_SIZE, x))
        self.robot_y = max(0, min(FIELD_SIZE, y))
        self.robot_angle = angle
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 计算绘图区域
        width = self.width()
        height = self.height()
        start_x = (width - FIELD_SIZE * CELL_SIZE) // 2
        start_y = (height - FIELD_SIZE * CELL_SIZE) // 2
        
        # 绘制格子
        for row in range(FIELD_SIZE):
            for col in range(FIELD_SIZE):
                x = start_x + col * CELL_SIZE
                y = start_y + row * CELL_SIZE
                
                if (row + col) % 2 == 0:
                    painter.fillRect(x, y, CELL_SIZE, CELL_SIZE, QColor(60, 60, 80))
                else:
                    painter.fillRect(x, y, CELL_SIZE, CELL_SIZE, QColor(45, 45, 65))
                
                painter.setPen(QPen(QColor(80, 80, 100), 1))
                painter.drawRect(x, y, CELL_SIZE, CELL_SIZE)
        
        # 绘制场地边界
        painter.setPen(QPen(QColor(150, 150, 180), 3))
        painter.drawRect(start_x - 2, start_y - 2, 
                        FIELD_SIZE * CELL_SIZE + 4, 
                        FIELD_SIZE * CELL_SIZE + 4)
        
        # 绘制尺寸标注 (cm)
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        font = QFont("Arial", 9)
        painter.setFont(font)
        
        painter.drawText(start_x + FIELD_SIZE * CELL_SIZE + 10, start_y + FIELD_SIZE * CELL_SIZE + 20, "X (cm)")
        painter.drawText(start_x - 25, start_y - 5, "Y (cm)")
        painter.drawText(start_x - 20, start_y + FIELD_SIZE * CELL_SIZE + 20, "(0,0)")
        painter.drawText(start_x + FIELD_SIZE * CELL_SIZE + 5, start_y - 5, f"({FIELD_SIZE*CELL_CM},{FIELD_SIZE*CELL_CM})")
        
        # ========== 修复1: Y坐标映射 ==========
        # 注意：屏幕坐标系中Y轴向下为正，所以需要翻转
        # 机器人格子坐标 (0~6)，屏幕坐标计算：
        # robot_center_x = start_x + self.robot_x * CELL_SIZE
        # robot_center_y = start_y + (FIELD_SIZE - self.robot_y) * CELL_SIZE
        # 当 robot_y = 0 时，显示在最底部（Y=0位置），正确！
        # 当 robot_y = 6 时，显示在最顶部（Y=360cm位置），正确！
        robot_center_x = start_x + self.robot_x * CELL_SIZE
        robot_center_y = start_y + (FIELD_SIZE - self.robot_y) * CELL_SIZE
        
        painter.setBrush(QBrush(QColor(255, 80, 80, 220)))
        painter.setPen(QPen(QColor(255, 200, 200), 2))
        painter.drawEllipse(QPointF(robot_center_x, robot_center_y), CELL_SIZE // 6, CELL_SIZE // 6)
        
        # ========== 修复2: 角度定义（Y正方向为0度） ==========
        # 角度定义：Y轴正方向为0度，顺时针为正，范围-180~180
        # 屏幕坐标系中：X向右为正，Y向下为正
        # 数学坐标系：X向右为正，Y向上为正
        # 我们需要将机器人的角度转换为屏幕上的显示角度
        
        # 步骤：
        # 1. 原始角度定义：Y正方向为0度，顺时针为正
        # 2. 在数学坐标系中（Y向上）：0度是向上(0,1)，顺时针是向右转
        # 3. 屏幕坐标系中（Y向下）：0度应该向上，但屏幕Y轴向下，所以需要取反
        
        # 方法：将角度取负，因为屏幕Y轴与数学Y轴相反
        # 同时，由于我们需要箭头指向机器人朝向，直接使用角度取负即可
        angle_rad = math.radians(-self.robot_angle)
        
        arrow_length = CELL_SIZE // 2
        arrow_end_x = robot_center_x + arrow_length * math.sin(angle_rad)
        arrow_end_y = robot_center_y - arrow_length * math.cos(angle_rad)
        
        painter.setPen(QPen(QColor(255, 100, 100), 3))
        painter.drawLine(QPointF(robot_center_x, robot_center_y), 
                        QPointF(arrow_end_x, arrow_end_y))
        
        arrow_size = 8
        # 箭头角度基于线的方向
        line_angle = math.atan2(arrow_end_y - robot_center_y, arrow_end_x - robot_center_x)
        arrow_angle1 = line_angle + math.radians(150)
        arrow_angle2 = line_angle - math.radians(150)
        
        p1 = QPointF(arrow_end_x + arrow_size * math.cos(arrow_angle1),
                    arrow_end_y + arrow_size * math.sin(arrow_angle1))
        p2 = QPointF(arrow_end_x + arrow_size * math.cos(arrow_angle2),
                    arrow_end_y + arrow_size * math.sin(arrow_angle2))
        
        painter.setBrush(QBrush(QColor(255, 100, 100)))
        painter.drawPolygon(QPolygonF([QPointF(arrow_end_x, arrow_end_y), p1, p2]))
        
        # 可选：绘制方向指示圆环（0度方向的参考线）
        painter.setPen(QPen(QColor(100, 100, 150), 1, Qt.DashLine))
        # 绘制向上的参考线（Y正方向）
        ref_end_x = robot_center_x
        ref_end_y = robot_center_y - CELL_SIZE // 2
        painter.drawLine(QPointF(robot_center_x, robot_center_y), QPointF(ref_end_x, ref_end_y))

class VexDebugger(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_worker = SerialWorker()
        
        self.data_buffers = {
            'time': deque(maxlen=MAX_POINTS),
            'ch1': deque(maxlen=MAX_POINTS),
            'ch2': deque(maxlen=MAX_POINTS),
            'ch3': deque(maxlen=MAX_POINTS),
            'ch4': deque(maxlen=MAX_POINTS)
        }
        
        self.is_recording = False
        self.record_file = None
        self.csv_writer = None
        self.is_merged = False
        self.show_map = False
        
        self.init_ui()
        self.init_signals()
        
    def init_ui(self):
        self.setWindowTitle("VEX V5 专业调试平台 - 全场定位版 v1.8")
        self.setGeometry(100, 100, 1400, 900)
        
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; }
            QGroupBox {
                color: #ffffff;
                border: 2px solid #555;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 8px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #45a049; }
            QLabel { color: #ffffff; }
            QLineEdit {
                background-color: #3c3c3c;
                color: #ffffff;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 4px;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # 工具栏
        toolbar = self.addToolBar("控制栏")
        toolbar.setStyleSheet("QToolBar { background-color: #3c3c3c; }")
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        toolbar.addWidget(QLabel("串口: "))
        toolbar.addWidget(self.port_combo)
        
        refresh_btn = QAction("刷新", self)
        refresh_btn.triggered.connect(self.refresh_ports)
        toolbar.addAction(refresh_btn)
        toolbar.addSeparator()
        
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['9600', '115200', '230400', '460800'])
        self.baud_combo.setCurrentText('115200')
        toolbar.addWidget(QLabel("波特率: "))
        toolbar.addWidget(self.baud_combo)
        toolbar.addSeparator()
        
        self.connect_btn = QAction("连接", self)
        self.connect_btn.triggered.connect(self.toggle_connection)
        toolbar.addAction(self.connect_btn)
        toolbar.addSeparator()
        
        self.merge_btn = QAction("合并", self)
        self.merge_btn.triggered.connect(self.toggle_merge_mode)
        toolbar.addAction(self.merge_btn)
        toolbar.addSeparator()
        
        self.map_btn = QAction("地图", self)
        self.map_btn.triggered.connect(self.toggle_map_mode)
        toolbar.addAction(self.map_btn)
        toolbar.addSeparator()
        
        self.record_btn = QAction("开始录制", self)
        self.record_btn.triggered.connect(self.toggle_recording)
        self.record_btn.setEnabled(False)
        toolbar.addAction(self.record_btn)
        
        screenshot_btn = QAction("截图", self)
        screenshot_btn.triggered.connect(self.take_screenshot)
        toolbar.addAction(screenshot_btn)
        toolbar.addSeparator()
        
        self.status_label = QLabel("未连接")
        toolbar.addWidget(self.status_label)
        
        # 图表区域
        self.chart_stacked_widget = QStackedWidget()
        
        # 曲线显示页面
        self.curve_widget = QWidget()
        curve_main_layout = QVBoxLayout(self.curve_widget)
        
        self.stacked_widget = QStackedWidget()
        
        self.separate_widget = QWidget()
        separate_layout = QGridLayout(self.separate_widget)
        
        self.separate_plots = []
        self.separate_curves = []
        colors = ['#ff6b6b', '#4ecdc4', '#45b7d1', '#96ceb4']
        names = ['通道1', '通道2', '通道3', '通道4']
        
        for i in range(4):
            plot = pg.PlotWidget()
            plot.setLabel('left', names[i], color='#fff')
            plot.setLabel('bottom', '时间 (秒)', color='#fff')
            plot.setTitle(names[i], color='#fff', size='12pt')
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setBackground('#1e1e1e')
            plot.setYRange(-60, 60)
            
            curve = plot.plot(pen=pg.mkPen(color=colors[i], width=2))
            
            self.separate_plots.append(plot)
            self.separate_curves.append(curve)
            
            row = i // 2
            col = i % 2
            separate_layout.addWidget(plot, row, col)
        
        self.merged_widget = QWidget()
        merged_layout = QVBoxLayout(self.merged_widget)
        
        self.merged_plot = pg.PlotWidget()
        self.merged_plot.setLabel('left', '数值', color='#fff')
        self.merged_plot.setLabel('bottom', '时间 (秒)', color='#fff')
        self.merged_plot.setTitle('4通道数据对比', color='#fff', size='12pt')
        self.merged_plot.showGrid(x=True, y=True, alpha=0.3)
        self.merged_plot.setBackground('#1e1e1e')
        self.merged_plot.setYRange(-60, 60)
        self.merged_plot.addLegend()
        
        self.merged_curves = []
        for i in range(4):
            curve = self.merged_plot.plot(
                pen=pg.mkPen(color=colors[i], width=2),
                name=names[i]
            )
            self.merged_curves.append(curve)
        
        merged_layout.addWidget(self.merged_plot)
        
        self.stacked_widget.addWidget(self.separate_widget)
        self.stacked_widget.addWidget(self.merged_widget)
        self.stacked_widget.setCurrentIndex(0)
        
        curve_main_layout.addWidget(self.stacked_widget)
        
        # 地图显示页面
        self.map_widget_container = QWidget()
        map_layout = QHBoxLayout(self.map_widget_container)
        
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        self.field_map = FieldMapWidget()
        left_layout.addWidget(self.field_map)
        
        right_panel = QWidget()
        right_panel.setMaximumWidth(800)
        right_layout = QVBoxLayout(right_panel)
        
        init_group = QGroupBox("机器人初始参数")
        init_layout = QFormLayout(init_group)
        init_layout.setSpacing(10)
        
        # 输入框 - 单位是cm
        self.init_x = QLineEdit()
        self.init_x.setPlaceholderText("0 ~ 360 cm ")
        self.init_x.setValidator(QDoubleValidator(0, FIELD_SIZE * CELL_CM, 1))
        init_layout.addRow("初始X坐标(cm):", self.init_x)
        
        self.init_y = QLineEdit()
        self.init_y.setPlaceholderText("0 ~ 360 cm ")
        self.init_y.setValidator(QDoubleValidator(0, FIELD_SIZE * CELL_CM, 1))
        init_layout.addRow("初始Y坐标(cm):", self.init_y)
        
        self.init_angle = QLineEdit()
        self.init_angle.setPlaceholderText("-180 ~ 180°")
        self.init_angle.setValidator(QDoubleValidator(-180, 180, 1))
        init_layout.addRow("初始朝向角度:", self.init_angle)
        
        apply_btn = QPushButton("应用初始参数")
        apply_btn.clicked.connect(self.apply_initial_params)
        init_layout.addRow("", apply_btn)
        
        send_init_btn = QPushButton("发送参数到V5")
        send_init_btn.setStyleSheet("background-color: #2196F3;")
        send_init_btn.clicked.connect(self.send_initial_params_to_vex)
        init_layout.addRow("", send_init_btn)
        
        init_group.setLayout(init_layout)
        right_layout.addWidget(init_group)
        
        pos_group = QGroupBox("实时位置反馈")
        pos_layout = QFormLayout(pos_group)
        
        self.real_x_label = QLabel("--")
        self.real_y_label = QLabel("--")
        self.real_angle_label = QLabel("--")
        
        pos_layout.addRow("当前X坐标(cm):", self.real_x_label)
        pos_layout.addRow("当前Y坐标(cm):", self.real_y_label)
        pos_layout.addRow("当前朝向:", self.real_angle_label)
        
        pos_group.setLayout(pos_layout)
        right_layout.addWidget(pos_group)
        
        right_layout.addStretch()
        
        map_layout.addWidget(left_panel, 3)
        map_layout.addWidget(right_panel, 1)
        
        self.chart_stacked_widget.addWidget(self.curve_widget)
        self.chart_stacked_widget.addWidget(self.map_widget_container)
        self.chart_stacked_widget.setCurrentIndex(0)
        
        # 下方数据面板
        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(self.chart_stacked_widget)
        
        data_panel = QWidget()
        data_layout = QHBoxLayout(data_panel)
        
        self.value_labels = []
        for i in range(4):
            group = QGroupBox(f"通道{i+1}")
            layout = QVBoxLayout()
            label = QLabel("0.00")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("font-size: 24px; font-weight: bold; color: #4CAF50;")
            layout.addWidget(label)
            group.setLayout(layout)
            data_layout.addWidget(group)
            self.value_labels.append(label)
        
        self.stats_text = QTextEdit()
        self.stats_text.setMaximumHeight(80)
        self.stats_text.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: monospace;")
        
        splitter.addWidget(data_panel)
        splitter.addWidget(self.stats_text)
        
        main_layout.addWidget(splitter)
        
        # 控制面板
        control_panel = QWidget()
        control_panel.setFixedHeight(180)
        control_layout = QHBoxLayout(control_panel)
        
        pid_group = QGroupBox("PID参数")
        pid_group.setFixedHeight(150)
        pid_layout = QHBoxLayout()
        pid_layout.setSpacing(15)
        
        pid1_widget = QWidget()
        pid1_layout = QHBoxLayout(pid1_widget)
        pid1_layout.setSpacing(5)
        
        pid1_label = QLabel("PID1")
        pid1_label.setStyleSheet("font-weight: bold; color: #ff6b6b;")
        pid1_label.setFixedWidth(72)
        
        self.pid1_kp = QLineEdit()
        self.pid1_kp.setPlaceholderText("Kp")
        self.pid1_kp.setFixedWidth(72)
        self.pid1_kp.setValidator(QDoubleValidator())
        
        self.pid1_ki = QLineEdit()
        self.pid1_ki.setPlaceholderText("Ki")
        self.pid1_ki.setFixedWidth(72)
        self.pid1_ki.setValidator(QDoubleValidator())
        
        self.pid1_kd = QLineEdit()
        self.pid1_kd.setPlaceholderText("Kd")
        self.pid1_kd.setFixedWidth(72)
        self.pid1_kd.setValidator(QDoubleValidator())
        
        pid1_layout.addWidget(pid1_label)
        pid1_layout.addWidget(self.pid1_kp)
        pid1_layout.addWidget(self.pid1_ki)
        pid1_layout.addWidget(self.pid1_kd)
        
        separator = QFrame()
        separator.setFrameShape(QFrame.VLine)
        separator.setFrameShadow(QFrame.Sunken)
        separator.setFixedWidth(2)
        separator.setStyleSheet("background-color: #555;")
        
        pid2_widget = QWidget()
        pid2_layout = QHBoxLayout(pid2_widget)
        pid2_layout.setSpacing(5)
        
        pid2_label = QLabel("PID2")
        pid2_label.setStyleSheet("font-weight: bold; color: #4ecdc4;")
        pid2_label.setFixedWidth(72)
        
        self.pid2_kp = QLineEdit()
        self.pid2_kp.setPlaceholderText("Kp")
        self.pid2_kp.setFixedWidth(72)
        self.pid2_kp.setValidator(QDoubleValidator())
        
        self.pid2_ki = QLineEdit()
        self.pid2_ki.setPlaceholderText("Ki")
        self.pid2_ki.setFixedWidth(72)
        self.pid2_ki.setValidator(QDoubleValidator())
        
        self.pid2_kd = QLineEdit()
        self.pid2_kd.setPlaceholderText("Kd")
        self.pid2_kd.setFixedWidth(72)
        self.pid2_kd.setValidator(QDoubleValidator())
        
        pid2_layout.addWidget(pid2_label)
        pid2_layout.addWidget(self.pid2_kp)
        pid2_layout.addWidget(self.pid2_ki)
        pid2_layout.addWidget(self.pid2_kd)
        
        send_pid_btn = QPushButton("send")
        send_pid_btn.setFixedSize(70, 32)
        send_pid_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                font-size: 20px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        send_pid_btn.clicked.connect(self.send_cascade_pid)
        
        pid_layout.addWidget(pid1_widget)
        pid_layout.addWidget(separator)
        pid_layout.addWidget(pid2_widget)
        pid_layout.addWidget(send_pid_btn)
        pid_layout.addStretch()
        
        pid_group.setLayout(pid_layout)
        control_layout.addWidget(pid_group, 3)
        
        cmd_group = QGroupBox("命令控制")
        cmd_group.setFixedHeight(150)
        cmd_layout = QVBoxLayout()
        cmd_layout.setSpacing(5)
        
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("输入命令...")
        self.cmd_input.setMinimumHeight(30)
        
        send_cmd_btn = QPushButton("发送")
        send_cmd_btn.setMinimumHeight(36)
        send_cmd_btn.clicked.connect(self.send_command)
        
        quick_cmd_layout = QHBoxLayout()
        quick_cmd_layout.setSpacing(5)
        
        reset_btn = QPushButton("复位")
        reset_btn.setFixedHeight(37)
        reset_btn.setStyleSheet("background-color: #555;")
        reset_btn.clicked.connect(lambda: self.cmd_input.setText("RESET"))
        
        status_btn = QPushButton("状态")
        status_btn.setFixedHeight(37)
        status_btn.setStyleSheet("background-color: #555;")
        status_btn.clicked.connect(lambda: self.cmd_input.setText("STATUS"))
        
        stop_btn = QPushButton("停止")
        stop_btn.setFixedHeight(37)
        stop_btn.setStyleSheet("background-color: #f44336;")
        stop_btn.clicked.connect(lambda: self.cmd_input.setText("STOP"))
        
        quick_cmd_layout.addWidget(reset_btn)
        quick_cmd_layout.addWidget(status_btn)
        quick_cmd_layout.addWidget(stop_btn)
        quick_cmd_layout.addStretch()
        
        cmd_layout.addWidget(self.cmd_input)
        cmd_layout.addWidget(send_cmd_btn)
        cmd_layout.addLayout(quick_cmd_layout)
        
        cmd_group.setLayout(cmd_layout)
        control_layout.addWidget(cmd_group, 2)
        
        main_layout.addWidget(control_panel)
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(50)
    
    def toggle_map_mode(self):
        self.show_map = not self.show_map
        if self.show_map:
            self.chart_stacked_widget.setCurrentIndex(1)
            self.map_btn.setText("曲线")
            self.status_label.setText("当前模式：场地地图")
        else:
            self.chart_stacked_widget.setCurrentIndex(0)
            self.map_btn.setText("地图")
            self.status_label.setText("当前模式：数据曲线")
    
    def apply_initial_params(self):
        """应用初始参数到地图显示 (输入是cm，内部转换为格子坐标)"""
        try:
            x_cm = float(self.init_x.text()) if self.init_x.text() else 180.0
            y_cm = float(self.init_y.text()) if self.init_y.text() else 180.0
            angle = float(self.init_angle.text()) if self.init_angle.text() else 0.0
            
            # 边界检查 (0~360cm)
            x_cm = max(0, min(FIELD_SIZE * CELL_CM, x_cm))
            y_cm = max(0, min(FIELD_SIZE * CELL_CM, y_cm))
            
            # cm转格子坐标
            x_grid = x_cm / CELL_CM
            y_grid = y_cm / CELL_CM
            
        except ValueError:
            self.status_label.setText("❌ 请输入有效的数值")
            return
        
        self.field_map.update_robot_position(x_grid, y_grid, angle)
        self.real_x_label.setText(f"{x_cm:.1f}")
        self.real_y_label.setText(f"{y_cm:.1f}")
        self.real_angle_label.setText(f"{angle:.1f}°")
        self.status_label.setText(f"✅ 初始参数已应用: X={x_cm}cm, Y={y_cm}cm, Angle={angle}")
    
    def send_initial_params_to_vex(self):
        """发送初始参数到V5主控 (发送cm单位)"""
        try:
            x_cm = float(self.init_x.text()) if self.init_x.text() else 180.0
            y_cm = float(self.init_y.text()) if self.init_y.text() else 180.0
            angle = float(self.init_angle.text()) if self.init_angle.text() else 0.0
            
            x_cm = max(0, min(FIELD_SIZE * CELL_CM, x_cm))
            y_cm = max(0, min(FIELD_SIZE * CELL_CM, y_cm))
            
        except ValueError:
            self.status_label.setText("❌ 请输入有效的数值")
            return
        
        cmd = f"INIT_POS,{x_cm:.1f},{y_cm:.1f},{angle:.1f}"
        self.serial_worker.send_command(cmd)
        self.status_label.setText(f"✅ 已发送初始参数: {cmd}")
        self.stats_text.append(f"[初始位置] X={x_cm}cm, Y={y_cm}cm, Angle={angle}")
    
    def update_robot_position(self, x_grid, y_grid, angle):
        """从VEX接收位置更新 (格子坐标)"""
        self.field_map.update_robot_position(x_grid, y_grid, angle)
        # 显示cm单位给用户
        x_cm = x_grid * CELL_CM
        y_cm = y_grid * CELL_CM
        self.real_x_label.setText(f"{x_cm:.1f}")
        self.real_y_label.setText(f"{y_cm:.1f}")
        self.real_angle_label.setText(f"{angle:.1f}°")
    
    def get_pid_value(self, line_edit):
        try:
            text = line_edit.text().strip()
            if text:
                return float(text)
            return None
        except ValueError:
            return None
    
    def send_cascade_pid(self):
        kp1 = self.get_pid_value(self.pid1_kp)
        ki1 = self.get_pid_value(self.pid1_ki)
        kd1 = self.get_pid_value(self.pid1_kd)
        kp2 = self.get_pid_value(self.pid2_kp)
        ki2 = self.get_pid_value(self.pid2_ki)
        kd2 = self.get_pid_value(self.pid2_kd)
        
        if None in [kp1, ki1, kd1, kp2, ki2, kd2]:
            self.status_label.setText("❌ 请填写所有PID参数")
            self.stats_text.append("[错误] PID参数未填写完整")
            return
        
        cmd = f"CASCADE_PID,{kp1:.2f},{ki1:.2f},{kd1:.2f},{kp2:.2f},{ki2:.2f},{kd2:.2f}"
        self.serial_worker.send_command(cmd)
        self.status_label.setText(f"✅ 已发送串级PID")
        self.stats_text.append(f"[PID] 外环({kp1},{ki1},{kd1}) 内环({kp2},{ki2},{kd2})")
    
    def toggle_merge_mode(self):
        self.is_merged = not self.is_merged
        if self.is_merged:
            self.stacked_widget.setCurrentIndex(1)
            self.merge_btn.setText("拆分")
            self.status_label.setText("当前模式：4通道合并显示")
        else:
            self.stacked_widget.setCurrentIndex(0)
            self.merge_btn.setText("合并")
            self.status_label.setText("当前模式：4通道分开显示")
    
    def init_signals(self):
        self.serial_worker.data_received.connect(self.on_data_received)
        self.serial_worker.status_changed.connect(self.on_status_changed)
        self.serial_worker.position_received.connect(self.on_position_received)
    
    def on_position_received(self, pos_data):
        self.update_robot_position(pos_data['x'], pos_data['y'], pos_data['angle'])
    
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
        if not ports:
            self.port_combo.addItem("未找到串口设备")
    
    def toggle_connection(self):
        if self.serial_worker.running:
            self.serial_worker.disconnect()
            self.connect_btn.setText("连接")
            self.record_btn.setEnabled(False)
        else:
            port_str = self.port_combo.currentText().split(' - ')[0]
            baudrate = int(self.baud_combo.currentText())
            if self.serial_worker.connect(port_str, baudrate):
                self.serial_worker.start()
                self.connect_btn.setText("断开")
                self.record_btn.setEnabled(True)
                for key in self.data_buffers:
                    self.data_buffers[key].clear()
    
    def on_data_received(self, data):
        self.data_buffers['time'].append(data['time'])
        self.data_buffers['ch1'].append(data['ch1'])
        self.data_buffers['ch2'].append(data['ch2'])
        self.data_buffers['ch3'].append(data['ch3'])
        self.data_buffers['ch4'].append(data['ch4'])
        
        self.value_labels[0].setText(f"{data['ch1']:.2f}")
        self.value_labels[1].setText(f"{data['ch2']:.2f}")
        self.value_labels[2].setText(f"{data['ch3']:.2f}")
        self.value_labels[3].setText(f"{data['ch4']:.2f}")
        
        if self.is_recording and self.csv_writer:
            self.csv_writer.writerow([
                data['time'], data['ch1'], data['ch2'], data['ch3'], data['ch4']
            ])
    
    def update_plots(self):
        if len(self.data_buffers['time']) < 2:
            return
        
        times = list(self.data_buffers['time'])
        
        if self.is_merged:
            for i in range(4):
                ch_key = f'ch{i+1}'
                values = list(self.data_buffers[ch_key])
                if len(values) == len(times):
                    self.merged_curves[i].setData(times, values)
            if times:
                self.merged_plot.setXRange(max(0, times[-1] - 10), times[-1] + 1)
        else:
            for i in range(4):
                ch_key = f'ch{i+1}'
                values = list(self.data_buffers[ch_key])
                if len(values) == len(times):
                    self.separate_curves[i].setData(times, values)
                    self.separate_plots[i].setXRange(max(0, times[-1] - 10), times[-1] + 1)
    
    def toggle_recording(self):
        if not self.is_recording:
            filename = f"data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.record_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.record_file)
            self.csv_writer.writerow(['时间(秒)', '通道1', '通道2', '通道3', '通道4'])
            self.is_recording = True
            self.record_btn.setText("停止录制")
            self.status_label.setText(f"录制中: {filename}")
        else:
            self.record_file.close()
            self.is_recording = False
            self.record_btn.setText("开始录制")
            self.status_label.setText("录制已停止")
    
    def take_screenshot(self):
        filename = f"screenshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        screen = QApplication.primaryScreen()
        screenshot = screen.grabWindow(self.winId())
        screenshot.save(filename)
        self.status_label.setText(f"截图已保存: {filename}")
    
    def send_command(self):
        cmd = self.cmd_input.text()
        if cmd:
            self.serial_worker.send_command(cmd)
            self.status_label.setText(f"✅ 已发送: {cmd}")
            self.stats_text.append(f"[命令] {cmd}")
            self.cmd_input.clear()
    
    def on_status_changed(self, status):
        self.status_label.setText(status)
    
    def closeEvent(self, event):
        self.serial_worker.disconnect()
        if self.is_recording:
            self.record_file.close()
        event.accept()

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    font = QFont("Microsoft YaHei", 9)
    app.setFont(font)
    
    window = VexDebugger()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()