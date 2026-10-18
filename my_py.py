#!/usr/bin/env python3
"""
VEX V5 调试上位机 debug.py
=========================
功能
----
1. 4 个 PID 调参页（angle / distance / Lspeed / Rspeed），每页画 goal vs actual
2. 左侧地图：车当前位置 + 朝向
3. 中间 robot_data[] 12 项（电机转速 + 定位轮 + 航向）
4. 右侧遥控器 16 项
5. 键盘 ←/→ 或工具栏按钮翻页

串口协议（与 debug.cpp 匹配）
------------------------------
POS:gx,gy,yaw
RD:v0,...,v11
PID:name,kp,ki,kd,goal,actual,error,last_error,
      kp_out,ki_out,kd_out,out,i_limit,out_limit
RC:ch1,ch2,ch3,ch4,btnA,btnB,btnX,btnY,btnUp,btnDown,
   btnLeft,btnRight,L1,L2,R1,R2
"""

import sys
import time
import math
from collections import deque

import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import pyqtgraph as pg

# OpenGL 加速
try:
    pg.setConfigOptions(useOpenGL=True, antialias=False)
except Exception:
    pass

# ---------------- 配置 ----------------
MAX_POINTS    = 600
DRAW_POINTS   = 300
FIELD_SIZE    = 6
CELL_SIZE     = 80
CELL_CM       = 60
UI_REFRESH_MS = 40

PID_NAMES = ["angle", "distance", "Lspeed", "Rspeed"]

PID_FIELDS = [
    ("kp",           "#ff6b6b"),
    ("ki",           "#ff9f43"),
    ("kd",           "#feca57"),
    ("goal",         "#4ecdc4"),
    ("actual",       "#45b7d1"),
    ("error",        "#e056fd"),
    ("last_error",   "#be2edd"),
    ("kp_output",    "#96ceb4"),
    ("ki_output",    "#2ecc71"),
    ("kd_output",    "#5f27cd"),
    ("output",       "#ffffff"),
    ("i_limit",      "#888888"),
    ("output_limit", "#555555"),
]

PLOT_FIELDS = [
    ("goal",   "#00e5ff"),
    ("actual", "#4d8cff"),
]

DISPLAY_FIELDS = [
    ("kp",      "#ff6b6b"),
    ("ki",      "#ff9f43"),
    ("kd",      "#feca57"),
    ("actual",  "#4d8cff"),
    ("goal",    "#00e5ff"),
    ("output",  "#ffffff"),
    ("i_limit", "#aaaaaa"),
]

ROBOT_FIELDS = [
    ("left_1",     "#ff6b6b"),
    ("left_2",     "#ff9f43"),
    ("left_3",     "#feca57"),
    ("left_4",     "#ff6b81"),
    ("right_1",    "#4ecdc4"),
    ("right_2",    "#45b7d1"),
    ("right_3",    "#5f27cd"),
    ("right_4",    "#341f97"),
    ("inertial-H", "#96ceb4"),
    ("inertial-R", "#2ecc71"),
    ("rotation-X", "#e056fd"),
    ("rotation-Y", "#be2edd"),
]


# ============ 串口线程 ============
class SerialWorker(QThread):
    pid_received = pyqtSignal(str, dict)
    pos_received = pyqtSignal(float, float, float)
    rc_received  = pyqtSignal(dict)
    rd_received  = pyqtSignal(list)
    status_changed = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.running = False

    def connect_port(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
            self.running = True
            self.status_changed.emit(f"已连接 {port}")
            return True
        except Exception as e:
            self.status_changed.emit(f"连接失败: {e}")
            return False

    def disconnect_port(self):
        self.running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.status_changed.emit("已断开")

    def run(self):
        try:
            import ctypes
            ctypes.windll.kernel32.SetThreadPriority(
                ctypes.windll.kernel32.GetCurrentThread(), 2)
        except Exception:
            pass

        buf = b""
        while self.running:
            if self.ser is None or not self.ser.is_open:
                self.msleep(10)
                continue
            try:
                n = self.ser.in_waiting
                if n:
                    buf += self.ser.read(n)
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        line = line.decode('utf-8', errors='ignore').strip()
                        if line:
                            self.parse_line(line)
                    if len(buf) > 4096:
                        buf = buf[-1024:]
                else:
                    self.msleep(1)
            except Exception:
                self.msleep(5)

    def parse_line(self, line):
        if line.startswith("PID:"):
            parts = line[4:].split(',')
            if len(parts) == 14:
                name = parts[0]
                try:
                    vals = [float(x) for x in parts[1:]]
                    data = {f: v for (f, _), v in zip(PID_FIELDS, vals)}
                    self.pid_received.emit(name, data)
                except ValueError:
                    pass
        elif line.startswith("POS:"):
            parts = line[4:].split(',')
            if len(parts) == 3:
                try:
                    self.pos_received.emit(*[float(x) for x in parts])
                except ValueError:
                    pass
        elif line.startswith("RD:"):
            parts = line[3:].split(',')
            if len(parts) == 12:
                try:
                    self.rd_received.emit([float(x) for x in parts])
                except ValueError:
                    pass
        elif line.startswith("RC:"):
            parts = line[3:].split(',')
            if len(parts) == 16:
                try:
                    v = [int(float(x)) for x in parts]
                    self.rc_received.emit({
                        'Ch1': v[0], 'Ch2': v[1], 'Ch3': v[2], 'Ch4': v[3],
                        'BtnA': bool(v[4]), 'BtnB': bool(v[5]),
                        'BtnX': bool(v[6]), 'BtnY': bool(v[7]),
                        'BtnUp': bool(v[8]), 'BtnDown': bool(v[9]),
                        'BtnLeft': bool(v[10]), 'BtnRight': bool(v[11]),
                        'L1': bool(v[12]), 'L2': bool(v[13]),
                        'R1': bool(v[14]), 'R2': bool(v[15]),
                    })
                except (ValueError, IndexError):
                    pass


# ============ 地图 ============
class FieldMap(QWidget):
    def __init__(self):
        super().__init__()
        self.rx = 180.0
        self.ry = 180.0
        self.ra = 0.0
        self.trail = deque(maxlen=300)
        self._need_repaint = False
        self.setMinimumSize(460, 460)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._repaint_if_needed)
        self.timer.start(100)

    def update_pos(self, x, y, a):
        self.rx = max(0.0, min(float(FIELD_SIZE * CELL_CM), float(x)))
        self.ry = max(0.0, min(float(FIELD_SIZE * CELL_CM), float(y)))
        self.ra = float(a)
        self.trail.append((self.rx, self.ry))
        self._need_repaint = True

    def clear_trail(self):
        self.trail.clear()
        self._need_repaint = True

    def _repaint_if_needed(self):
        if self._need_repaint:
            self._need_repaint = False
            self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.Antialiasing)
            W = FIELD_SIZE * CELL_SIZE
            ox = (self.width() - W) // 2
            oy = (self.height() - W) // 2

            # 棋盘格
            for row in range(FIELD_SIZE):
                for col in range(FIELD_SIZE):
                    x = ox + col * CELL_SIZE
                    y = oy + row * CELL_SIZE
                    c = QColor(55, 55, 75) if (row + col) % 2 == 0 else QColor(40, 40, 58)
                    p.fillRect(x, y, CELL_SIZE, CELL_SIZE, c)
                    p.setPen(QPen(QColor(75, 75, 95), 1))
                    p.drawRect(x, y, CELL_SIZE, CELL_SIZE)

            p.setPen(QPen(QColor(140, 140, 170), 3))
            p.drawRect(ox - 2, oy - 2, W + 4, W + 4)

            # 坐标轴标注
            p.setPen(QPen(QColor(200, 200, 200), 1))
            p.setFont(QFont("Arial", 9))
            p.drawText(ox + W + 8, oy + W + 16, "X (cm)")
            p.drawText(ox - 20, oy - 6, "Y (cm)")
            p.drawText(ox - 20, oy + W + 16, "(0,0)")

            # 轨迹
            if len(self.trail) > 1:
                p.setPen(QPen(QColor(0, 200, 255, 180), 2))
                pts = [QPointF(
                            ox + (tx / CELL_CM) * CELL_SIZE,
                            oy + ((FIELD_SIZE * CELL_CM - ty) / CELL_CM) * CELL_SIZE)
                       for (tx, ty) in self.trail]
                for i in range(len(pts) - 1):
                    p.drawLine(pts[i], pts[i + 1])

            # 车体
            px = ox + (self.rx / CELL_CM) * CELL_SIZE
            py = oy + ((FIELD_SIZE * CELL_CM - self.ry) / CELL_CM) * CELL_SIZE

            p.setBrush(QBrush(QColor(255, 80, 80, 220)))
            p.setPen(QPen(QColor(255, 200, 200), 2))
            p.drawEllipse(QPointF(px, py), CELL_SIZE // 5, CELL_SIZE // 5)

            rad = math.radians(-self.ra)
            L = CELL_SIZE // 2
            ex = px + L * math.sin(rad)
            ey = py - L * math.cos(rad)

            p.setPen(QPen(QColor(255, 100, 100), 3))
            p.drawLine(QPointF(px, py), QPointF(ex, ey))

            line_ang = math.atan2(ey - py, ex - px)
            a1 = line_ang + math.radians(150)
            a2 = line_ang - math.radians(150)
            s = 9
            p1 = QPointF(ex + s * math.cos(a1), ey + s * math.sin(a1))
            p2 = QPointF(ex + s * math.cos(a2), ey + s * math.sin(a2))
            p.setBrush(QBrush(QColor(255, 100, 100)))
            p.drawPolygon(QPolygonF([QPointF(ex, ey), p1, p2]))

            p.setPen(QPen(QColor(0, 255, 0), 1))
            p.setFont(QFont("Arial", 10, QFont.Bold))
            p.drawText(QPointF(px + 14, py - 14),
                       f"({self.rx:.0f},{self.ry:.0f}) {self.ra:.0f}°")
        finally:
            p.end()


# ============ 单个 PID 页 ============
class PidPage(QWidget):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.bufs = {f: deque(maxlen=MAX_POINTS) for (f, _) in PID_FIELDS}
        self.time_buf = deque(maxlen=MAX_POINTS)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        title = QLabel(f"PID: {name}    （goal 目标 vs actual 实际）")
        title.setStyleSheet(
            "font-size:18px; font-weight:bold; color:#00e5ff; padding:4px;")
        layout.addWidget(title)

        self.plot = pg.PlotWidget()
        self.plot.setBackground('#101010')
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel('left', 'value', color='#ddd')
        self.plot.setLabel('bottom', 'time (s)', color='#ddd')
        self.plot.addLegend(offset=(10, 10))
        self.plot.setMouseEnabled(x=False, y=False)
        self.plot.setMenuEnabled(False)
        self.plot.setDownsampling(auto=True, mode='peak')
        self.plot.setClipToView(True)

        self.curves = {}
        for field, col in PLOT_FIELDS:
            self.curves[field] = self.plot.plot(
                pen=pg.mkPen(col, width=3), name=field)
        layout.addWidget(self.plot, 3)

        # 数值表
        grid = QGridLayout()
        grid.setSpacing(8)
        self.value_labels = {}
        for i, (field, col) in enumerate(DISPLAY_FIELDS):
            lb_name = QLabel(field)
            lb_name.setStyleSheet(f"color:{col}; font-size:13px; padding:2px;")
            lb_val = QLabel("--")
            lb_val.setStyleSheet(
                "color:#fff; background:#1a1a1a; border:1px solid #444; "
                "border-radius:3px; padding:6px 12px; font-weight:bold; "
                "font-size:17px;")
            lb_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row = i // 2
            cidx = (i % 2) * 2
            grid.addWidget(lb_name, row, cidx)
            grid.addWidget(lb_val, row, cidx + 1)
            self.value_labels[field] = lb_val
        layout.addLayout(grid, 2)

    def push(self, t, values):
        self.time_buf.append(t)
        for f in self.bufs:
            v = values.get(f, 0.0)
            self.bufs[f].append(v)
            if f in self.value_labels:
                self.value_labels[f].setText(f"{v:.2f}")

    def refresh(self):
        if len(self.time_buf) < 2:
            return
        t_all = list(self.time_buf)
        n = min(len(t_all), DRAW_POINTS)
        t_ds = t_all[-n:]
        for f, c in self.curves.items():
            y_all = list(self.bufs[f])
            y_ds = y_all[-n:]
            if len(y_ds) == len(t_ds):
                c.setData(t_ds, y_ds)
        self.plot.setXRange(t_ds[-1] - 10, t_ds[-1] + 0.3, padding=0)

    def clear(self):
        self.time_buf.clear()
        for b in self.bufs.values():
            b.clear()
        for lb in self.value_labels.values():
            lb.setText("--")


# ============ robot_data 数值条 ============
class RobotBar(QWidget):
    def __init__(self):
        super().__init__()
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.labels = {}
        for i, (field, col) in enumerate(ROBOT_FIELDS):
            lb_name = QLabel(field)
            lb_name.setStyleSheet(f"color:{col}; font-size:11px;")
            lb_val = QLabel("--")
            lb_val.setStyleSheet(
                "color:#fff; background:#1a1a1a; border:1px solid #444; "
                "border-radius:2px; padding:2px 6px; "
                "font-weight:bold; font-size:12px;")
            lb_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row = i // 6
            cidx = (i % 6) * 2
            layout.addWidget(lb_name, row, cidx)
            layout.addWidget(lb_val, row, cidx + 1)
            self.labels[field] = lb_val

    def push(self, values):
        for i, (f, _) in enumerate(ROBOT_FIELDS):
            self.labels[f].setText(f"{values[i]:.1f}")

    def clear(self):
        for lb in self.labels.values():
            lb.setText("--")


# ============ 主窗口 ============
class VexDebugger(QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = SerialWorker()
        self.t0 = time.time()
        self.rc_widgets = {}
        self.cur_page = 0

        self.pid_pages = {name: PidPage(name) for name in PID_NAMES}
        self.robot_bar = RobotBar()

        self.init_ui()

        self.worker.pid_received.connect(self.on_pid)
        self.worker.pos_received.connect(self.on_pos)
        self.worker.rc_received.connect(self.on_rc)
        self.worker.rd_received.connect(self.on_rd)
        self.worker.status_changed.connect(self.on_status)

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_cycle)
        self.timer.start(UI_REFRESH_MS)

    def init_ui(self):
        self.setWindowTitle("VEX V5 调试上位机 debug.py")
        self.resize(1500, 950)
        self.setStyleSheet("""
            QMainWindow { background:#1a1a1a; }
            QGroupBox { color:#fff; border:2px solid #333; border-radius:6px;
                        margin-top:12px; font-weight:bold; }
            QGroupBox::title { left:12px; padding:0 6px; color:#00e5ff; }
            QLabel { color:#fff; }
        """)

        tb = self.addToolBar("控制")
        tb.setStyleSheet("QToolBar { background:#252525; padding:4px; "
                         "border-bottom:1px solid #333; }")
        tb.setMovable(False)

        self.port_combo = QComboBox()
        self.refresh_ports()
        tb.addWidget(QLabel("串口:"))
        tb.addWidget(self.port_combo)

        act_ref = QAction("刷新", self)
        act_ref.triggered.connect(self.refresh_ports)
        tb.addAction(act_ref)
        tb.addSeparator()

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['115200', '230400', '460800'])
        tb.addWidget(QLabel("波特率:"))
        tb.addWidget(self.baud_combo)
        tb.addSeparator()

        self.act_conn = QAction("连接", self)
        self.act_conn.triggered.connect(self.toggle_conn)
        tb.addAction(self.act_conn)
        tb.addSeparator()

        self.act_prev = QAction("◀ 上一页", self)
        self.act_prev.triggered.connect(self.prev_page)
        tb.addAction(self.act_prev)

        self.page_label = QLabel(f"  1/{len(PID_NAMES)}  ")
        self.page_label.setStyleSheet(
            "color:#00e5ff; font-weight:bold; font-size:16px; "
            "padding:0 12px; min-width:160px; text-align:center;")
        self.page_label.setAlignment(Qt.AlignCenter)
        tb.addWidget(self.page_label)

        self.act_next = QAction("下一页 ▶", self)
        self.act_next.triggered.connect(self.next_page)
        tb.addAction(self.act_next)
        tb.addSeparator()

        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet("color:#4CAF50; padding:0 8px;")
        tb.addWidget(self.status_label)

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ---- 左：地图 ----
        left = QWidget()
        left.setFixedWidth(500)
        lv = QVBoxLayout(left)
        lv.setSpacing(8)

        map_group = QGroupBox("全局坐标地图")
        mv = QVBoxLayout(map_group)
        self.map = FieldMap()
        mv.addWidget(self.map)
        lv.addWidget(map_group)

        pos_group = QGroupBox("全局坐标 (cm)")
        pv = QFormLayout(pos_group)
        self.lbl_gx = QLabel("--")
        self.lbl_gy = QLabel("--")
        self.lbl_yaw = QLabel("--")
        for lb in (self.lbl_gx, self.lbl_gy, self.lbl_yaw):
            lb.setStyleSheet("font-size:18px; font-weight:bold; color:#00e5ff;")
        pv.addRow("gxT:", self.lbl_gx)
        pv.addRow("gyT:", self.lbl_gy)
        pv.addRow("yaw:", self.lbl_yaw)
        lv.addWidget(pos_group)
        lv.addStretch()
        root.addWidget(left)

        # ---- 中：robot_data + PID 页 ----
        mid = QWidget()
        mv2 = QVBoxLayout(mid)
        mv2.setContentsMargins(0, 0, 0, 0)
        mv2.setSpacing(8)

        rd_group = QGroupBox("robot_data[] 调试变量")
        rv = QVBoxLayout(rd_group)
        rv.addWidget(self.robot_bar)
        mv2.addWidget(rd_group)

        self.stack = QStackedWidget()
        for name in PID_NAMES:
            self.stack.addWidget(self.pid_pages[name])
        mv2.addWidget(self.stack, 1)
        root.addWidget(mid, 3)

        # ---- 右：遥控器 ----
        right = QWidget()
        right.setFixedWidth(340)
        rv2 = QVBoxLayout(right)
        rc_group = QGroupBox("遥控器数据 (ControllerData)")
        rv2.addWidget(rc_group)
        self.rc_grid = QGridLayout(rc_group)
        self.build_rc_table()
        rv2.addStretch()
        root.addWidget(right)

        QShortcut(QKeySequence(Qt.Key_Left),  self, self.prev_page)
        QShortcut(QKeySequence(Qt.Key_Right), self, self.next_page)

        self.update_page()

    def build_rc_table(self):
        fields = [
            ("Ch1", "int"),  ("Ch2", "int"),
            ("Ch3", "int"),  ("Ch4", "int"),
            ("BtnA", "bool"), ("BtnB", "bool"),
            ("BtnX", "bool"), ("BtnY", "bool"),
            ("BtnUp", "bool"), ("BtnDown", "bool"),
            ("BtnLeft", "bool"), ("BtnRight", "bool"),
            ("L1", "bool"), ("L2", "bool"),
            ("R1", "bool"), ("R2", "bool"),
        ]
        for col, h in enumerate(["变量", "类型", "当前值"]):
            lb = QLabel(h)
            lb.setStyleSheet("font-weight:bold; color:#00e5ff; padding:4px;")
            self.rc_grid.addWidget(lb, 0, col)

        for row, (name, typ) in enumerate(fields, start=1):
            n = QLabel(name); n.setStyleSheet("padding:4px;")
            t = QLabel(typ); t.setStyleSheet("color:#888; padding:4px;")
            v = QLabel("--")
            v.setAlignment(Qt.AlignCenter)
            v.setStyleSheet(
                "color:#4CAF50; font-weight:bold; font-size:14px; "
                "background:#1a1a1a; border:1px solid #444; "
                "border-radius:3px; padding:3px;")
            self.rc_grid.addWidget(n, row, 0)
            self.rc_grid.addWidget(t, row, 1)
            self.rc_grid.addWidget(v, row, 2)
            self.rc_widgets[name] = v

    def prev_page(self):
        self.cur_page = (self.cur_page - 1) % len(PID_NAMES)
        self.update_page()

    def next_page(self):
        self.cur_page = (self.cur_page + 1) % len(PID_NAMES)
        self.update_page()

    def update_page(self):
        self.stack.setCurrentIndex(self.cur_page)
        self.page_label.setText(
            f"  {self.cur_page + 1}/{len(PID_NAMES)}   "
            f"[ {PID_NAMES[self.cur_page]} ]")
        self.pid_pages[PID_NAMES[self.cur_page]].refresh()

    def on_pid(self, name, values):
        if name not in self.pid_pages:
            return
        t = time.time() - self.t0
        self.pid_pages[name].push(t, values)

    def on_rd(self, values):
        self.robot_bar.push(values)

    def on_pos(self, gx, gy, yaw):
        self.map.update_pos(gx, gy, yaw)
        self.lbl_gx.setText(f"{gx:.1f}")
        self.lbl_gy.setText(f"{gy:.1f}")
        self.lbl_yaw.setText(f"{yaw:.1f}°")

    def on_rc(self, d):
        for name, val in d.items():
            if name not in self.rc_widgets:
                continue
            lb = self.rc_widgets[name]
            if isinstance(val, bool):
                lb.setText("●" if val else "○")
                color = "#4CAF50" if val else "#666"
            else:
                lb.setText(str(val))
                color = "#4CAF50" if val != 0 else "#888"
            lb.setStyleSheet(
                f"color:{color}; font-weight:bold; font-size:14px; "
                f"background:#1a1a1a; border:1px solid #444; "
                f"border-radius:3px; padding:3px;")

    def on_status(self, s):
        self.status_label.setText(s)

    def refresh_cycle(self):
        self.pid_pages[PID_NAMES[self.cur_page]].refresh()

    def refresh_ports(self):
        self.port_combo.clear()
        try:
            ports = serial.tools.list_ports.comports()
        except Exception:
            ports = []
        if ports:
            for p in ports:
                self.port_combo.addItem(f"{p.device} - {p.description}")
        else:
            self.port_combo.addItem("无串口")

    def toggle_conn(self):
        if self.worker.running:
            self.worker.disconnect_port()
            if not self.worker.wait(500):
                self.worker.terminate()
                self.worker.wait(200)
            self.act_conn.setText("连接")
            self.status_label.setText("已断开")
        else:
            text = self.port_combo.currentText().split(' - ')[0].strip()
            if not text or text == "无串口":
                self.status_label.setText("❌ 请先刷新并选择串口")
                return
            try:
                baud = int(self.baud_combo.currentText())
            except ValueError:
                baud = 115200
            if self.worker.connect_port(text, baud):
                self.worker.start()
                self.act_conn.setText("断开")
                for page in self.pid_pages.values():
                    page.clear()
                self.robot_bar.clear()
                self.map.clear_trail()
                self.t0 = time.time()

    def closeEvent(self, e):
        self.worker.disconnect_port()
        if not self.worker.wait(500):
            self.worker.terminate()
            self.worker.wait(200)
        e.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setFont(QFont("Microsoft YaHei", 9))
    w = VexDebugger()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()