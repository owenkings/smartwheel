import argparse
import math
import os
import time
import shlex
import signal
import subprocess
import sys
from pathlib import Path
from typing import Dict, Optional, Tuple

from PyQt5 import QtCore, QtGui, QtWidgets

from wheelchair_navigation.semantic_map_store import default_semantic_map_path
from wheelchair_ui.mapping_manager import CONFLICTING_NAV_NODES, MappingManager, find_workspace_root
from wheelchair_ui.ros_bridge import RosBridge, default_named_goals_path


COLORS = {
    "bg": "#eef2f6",
    "panel": "#ffffff",
    "ink": "#17202a",
    "muted": "#667085",
    "line": "#d9e0e8",
    "blue": "#2f80ed",
    "green": "#168a56",
    "amber": "#b7791f",
    "red": "#c0392b",
    "dark": "#243447",
}


MANAGED_BACKEND_PATTERNS = (
    "ros2 launch wheelchair_bringup full_system.launch.py",
    "ros2 launch wheelchair_bringup mapping.launch.py",
    "ros2 launch wheelchair_3d_mapping rtabmap_3d_mapping.launch.py",
    "/robot_state_publisher/robot_state_publisher",
    "/wheelchair_sensors/xtm60_adapter_node",
    "/wheelchair_sensors/imu_adapter_node",
    "/wheelchair_sensors/ultrasonic_adapter_node",
    "/wheelchair_sensors/camera_adapter_node",
    "/wheelchair_perception/pointcloud_to_laserscan_node",
    "/wheelchair_perception/passability_analyzer_node",
    "/wheelchair_safety/safety_supervisor_node",
    "/wheelchair_diagnostics/localization_health_node",
    "/wheelchair_diagnostics/sensor_watchdog_node",
    "/wheelchair_navigation/goal_manager_node",
    "/wheelchair_navigation/navigation_status_node",
    "/wheelchair_base/zlac8030_driver_node",
    "/nav2_map_server/map_server",
    "/nav2_amcl/amcl",
    "/nav2_planner/planner_server",
    "/nav2_controller/controller_server",
    "/nav2_smoother/smoother_server",
    "/nav2_behaviors/behavior_server",
    "/nav2_bt_navigator/bt_navigator",
    "/nav2_waypoint_follower/waypoint_follower",
    "/nav2_velocity_smoother/velocity_smoother",
    "/nav2_lifecycle_manager/lifecycle_manager",
    "/robot_localization/ekf_node",
    "/slam_toolbox/async_slam_toolbox_node",
    "/rviz2/rviz2",
)


KNOWN_BACKEND_NODES = {
    "/xtm60_adapter_node",
    "/imu_adapter_node",
    "/ultrasonic_adapter_node",
    "/camera_adapter_node",
    "/zlac8030_driver_node",
    "/sensor_watchdog_node",
    "/robot_state_publisher",
    "/pointcloud_to_laserscan_node",
    "/map_server",
    "/amcl",
    "/planner_server",
    "/controller_server",
    "/smoother_server",
    "/behavior_server",
    "/bt_navigator",
    "/waypoint_follower",
    "/velocity_smoother",
    "/lifecycle_manager_localization",
    "/lifecycle_manager_navigation",
    "/ekf_filter_node",
    "/passability_analyzer_node",
    "/safety_supervisor_node",
    "/localization_health_node",
    "/goal_manager_node",
    "/navigation_status_node",
    "/slam_toolbox",
    "/rviz2",
}


def state_color(value: str) -> str:
    text = str(value or "").upper()
    if "EMERGENCY" in text or "ERROR" in text or "FAIL" in text or "LOST" in text:
        return COLORS["red"]
    if "WARN" in text or "DEGRADED" in text or "UNKNOWN" in text:
        return COLORS["amber"]
    if "OK" in text or "CLEAR" in text or "IDLE" in text or "ACTIVE" in text:
        return COLORS["green"]
    return COLORS["muted"]


def fmt_float(value, digits=2) -> str:
    try:
        number = float(value)
    except Exception:
        return "--"
    if not math.isfinite(number):
        return "--"
    return f"{number:.{digits}f}"


class StatusPill(QtWidgets.QLabel):
    def __init__(self, text="UNKNOWN"):
        super().__init__(text)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumHeight(32)
        self.set_state(text)

    def set_state(self, text: str):
        color = state_color(text)
        self.setText(str(text or "UNKNOWN"))
        self.setStyleSheet(
            f"""
            QLabel {{
                color: {color};
                background: #ffffff;
                border: 1px solid {color};
                border-radius: 7px;
                padding: 5px 10px;
                font-weight: 700;
            }}
            """
        )


class MetricCard(QtWidgets.QFrame):
    def __init__(self, title: str, value: str = "--"):
        super().__init__()
        self.setObjectName("metricCard")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(3)
        self.title = QtWidgets.QLabel(title)
        self.title.setObjectName("metricTitle")
        self.value = QtWidgets.QLabel(value)
        self.value.setObjectName("metricValue")
        self.value.setWordWrap(True)
        layout.addWidget(self.title)
        layout.addWidget(self.value)

    def set_value(self, value: str):
        self.value.setText(value)


class MapCanvas(QtWidgets.QWidget):
    point_clicked = QtCore.pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(620, 420)
        self.map_data: Optional[Dict] = None
        self.goals: Dict = {}
        self.semantic: Dict = {}
        self.pose: Dict = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._image: Optional[QtGui.QImage] = None
        self._layout: Optional[Tuple[float, float, float]] = None

    def set_map(self, data: Dict):
        self.map_data = data
        self._image = self._make_map_image(data)
        self.update()

    def set_goals(self, goals: Dict):
        self.goals = goals or {}
        self.update()

    def set_semantic(self, semantic: Dict):
        self.semantic = semantic or {}
        self.update()

    def set_pose(self, pose: Dict):
        self.pose = pose or {}
        self.update()

    def _make_map_image(self, data: Dict) -> QtGui.QImage:
        width = int(data.get("width", 1))
        height = int(data.get("height", 1))
        values = data.get("data", [])
        image = QtGui.QImage(width, height, QtGui.QImage.Format_RGB32)
        for y in range(height):
            for x in range(width):
                index = y * width + x
                value = values[index] if index < len(values) else -1
                color = 210 if value < 0 else (34 if value > 50 else 248)
                image.setPixel(x, y, QtGui.QColor(color, color, color).rgb())
        return image

    def _map_layout(self) -> Optional[Tuple[float, float, float]]:
        if not self.map_data:
            return None
        width = max(1, int(self.map_data.get("width", 1)))
        height = max(1, int(self.map_data.get("height", 1)))
        scale = min(self.width() / width, self.height() / height) * 0.96
        ox = (self.width() - width * scale) / 2.0
        oy = (self.height() - height * scale) / 2.0
        self._layout = (scale, ox, oy)
        return self._layout

    def world_to_canvas(self, x: float, y: float) -> Optional[QtCore.QPointF]:
        if not self.map_data:
            return None
        layout = self._map_layout()
        if not layout:
            return None
        scale, ox, oy = layout
        resolution = float(self.map_data.get("resolution", 0.05))
        origin = self.map_data.get("origin", {"x": 0.0, "y": 0.0})
        mx = (x - float(origin.get("x", 0.0))) / resolution
        my = float(self.map_data.get("height", 1)) - (y - float(origin.get("y", 0.0))) / resolution
        return QtCore.QPointF(ox + mx * scale, oy + my * scale)

    def canvas_to_world(self, point: QtCore.QPoint) -> Optional[Tuple[float, float]]:
        if not self.map_data:
            return None
        layout = self._map_layout()
        if not layout:
            return None
        scale, ox, oy = layout
        resolution = float(self.map_data.get("resolution", 0.05))
        width = float(self.map_data.get("width", 1))
        height = float(self.map_data.get("height", 1))
        mx = (point.x() - ox) / scale
        my = height - (point.y() - oy) / scale
        if mx < 0 or my < 0 or mx > width or my > height:
            return None
        origin = self.map_data.get("origin", {"x": 0.0, "y": 0.0})
        return (
            float(origin.get("x", 0.0)) + mx * resolution,
            float(origin.get("y", 0.0)) + my * resolution,
        )

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#e7ecf2"))
        layout = self._map_layout()
        if not self.map_data or not self._image or not layout:
            painter.setPen(QtGui.QColor(COLORS["muted"]))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "NO MAP")
            return

        scale, ox, oy = layout
        width = self.map_data["width"] * scale
        height = self.map_data["height"] * scale
        target = QtCore.QRectF(ox, oy, width, height)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        painter.drawImage(target, self._image)
        painter.setPen(QtGui.QPen(QtGui.QColor("#1f2937"), 1))
        painter.drawRect(target)

        self._draw_semantic(painter)
        self._draw_goals(painter)
        self._draw_pose(painter)

    def _draw_semantic(self, painter: QtGui.QPainter):
        for room in self.semantic.get("rooms", []):
            points = [self.world_to_canvas(x, y) for x, y in room.get("polygon", [])]
            points = [point for point in points if point is not None]
            if len(points) < 3:
                continue
            polygon = QtGui.QPolygonF(points)
            color = QtGui.QColor(room.get("color", "#2f80ed"))
            fill = QtGui.QColor(color)
            fill.setAlpha(38)
            painter.setBrush(fill)
            painter.setPen(QtGui.QPen(color, 2))
            painter.drawPolygon(polygon)

        for zone in self.semantic.get("no_go_zones", []):
            points = [self.world_to_canvas(x, y) for x, y in zone.get("polygon", [])]
            points = [point for point in points if point is not None]
            if len(points) < 3:
                continue
            polygon = QtGui.QPolygonF(points)
            fill = QtGui.QColor(COLORS["red"])
            fill.setAlpha(42)
            painter.setBrush(fill)
            painter.setPen(QtGui.QPen(QtGui.QColor(COLORS["red"]), 2))
            painter.drawPolygon(polygon)

    def _draw_goals(self, painter: QtGui.QPainter):
        painter.setFont(QtGui.QFont("Sans", 9, QtGui.QFont.Bold))
        for key, goal in self.goals.items():
            position = goal.get("position", [0.0, 0.0])
            point = self.world_to_canvas(float(position[0]), float(position[1]))
            if point is None:
                continue
            painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 2))
            painter.setBrush(QtGui.QColor(COLORS["green"]))
            painter.drawEllipse(point, 6, 6)
            painter.setPen(QtGui.QColor(COLORS["ink"]))
            painter.drawText(point + QtCore.QPointF(8, -8), goal.get("label", key))

    def _draw_pose(self, painter: QtGui.QPainter):
        x = float(self.pose.get("x", 0.0))
        y = float(self.pose.get("y", 0.0))
        yaw = float(self.pose.get("yaw", 0.0))
        point = self.world_to_canvas(x, y)
        if point is None:
            return
        painter.setBrush(QtGui.QColor(COLORS["blue"]))
        painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 2))
        painter.drawEllipse(point, 8, 8)
        end = QtCore.QPointF(point.x() + math.cos(yaw) * 24, point.y() - math.sin(yaw) * 24)
        painter.setPen(QtGui.QPen(QtGui.QColor(COLORS["blue"]), 3))
        painter.drawLine(point, end)

    def mousePressEvent(self, event):
        world = self.canvas_to_world(event.pos())
        if world is not None:
            self.point_clicked.emit(world[0], world[1])


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, bridge: RosBridge, mapping: MappingManager):
        super().__init__()
        self.bridge = bridge
        self.mapping = mapping
        self.workspace_root = find_workspace_root()
        self.system_process: Optional[QtCore.QProcess] = None
        self.system_pgid: Optional[int] = None
        self.system_launch_detail = ""
        self.system_log_tail = []
        self.closing = False
        # Used to suppress the periodic status refresh from auto-flipping the
        # run button back to "running" right after the user triggered a stop.
        # After an explicit stop, ROS topics may still appear briefly online
        # in cached status snapshots; we want the button to honour the user
        # intent for at least a few seconds before deciding the system is
        # actually running again.
        self._last_explicit_stop_at: float = 0.0
        self._stop_grace_seconds: float = 8.0
        self.latest_status: Dict = {}
        self.latest_goals: Dict = {}
        self.latest_map: Dict = {}
        self.setWindowTitle("SmartWheel")
        self.resize(1280, 820)
        self._build_ui()
        self._apply_styles()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh_all)
        self.timer.start(1000)
        self.refresh_all()
        # One-time initial state of the run button: if the systemd service is
        # already up (typical at boot via auto-start), show "已运行" so the
        # button reflects reality. After this, the button only changes when
        # the user clicks it.
        QtCore.QTimer.singleShot(500, self._init_run_button_state)

    def _init_run_button_state(self):
        if self._smartwheel_service_active() or self._status_indicates_system_active(
            self.latest_status
        ):
            self._set_run_state("running", "已运行", "")
        else:
            self._set_run_state("idle", "运行", "")

    def _build_ui(self):
        root = QtWidgets.QWidget()
        self.root_widget = root
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(14, 14, 14, 14)
        outer.setSpacing(12)

        top = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("SmartWheel")
        title.setObjectName("appTitle")
        top.addWidget(title)
        top.addStretch(1)
        self.run_btn = QtWidgets.QPushButton("运行")
        self.run_btn.setObjectName("runButton")
        self.run_btn.setProperty("runtimeState", "idle")
        self.run_btn.clicked.connect(self.start_system)
        self.nav_pill = StatusPill("IDLE")
        top.addWidget(self.run_btn)
        top.addWidget(self.nav_pill)
        self.settings_btn = QtWidgets.QPushButton("设置")
        self.settings_btn.clicked.connect(self.toggle_settings)
        top.addWidget(self.settings_btn)
        outer.addLayout(top)

        commands = QtWidgets.QHBoxLayout()
        self.stop_btn = QtWidgets.QPushButton("停止")
        self.stop_btn.setObjectName("dangerButton")
        self.resume_btn = QtWidgets.QPushButton("继续")
        self.zero_btn = QtWidgets.QPushButton("零速")
        self.shutdown_btn = QtWidgets.QPushButton("关闭硬件")
        self.shutdown_btn.setObjectName("outlineDangerButton")
        for button in (self.stop_btn, self.resume_btn, self.zero_btn, self.shutdown_btn):
            button.setMinimumHeight(46)
            commands.addWidget(button)
        outer.addLayout(commands)
        self.stop_btn.clicked.connect(lambda: self.bridge.node.set_software_stop(True))
        self.resume_btn.clicked.connect(lambda: self.bridge.node.set_software_stop(False))
        self.zero_btn.clicked.connect(self.bridge.node.publish_zero_velocity)
        self.shutdown_btn.clicked.connect(self.shutdown_hardware)

        body = QtWidgets.QHBoxLayout()
        body.setSpacing(12)
        self.map_canvas = MapCanvas()
        self.map_canvas.point_clicked.connect(self.fill_goal_from_map)
        body.addWidget(self.map_canvas, 1)

        right = QtWidgets.QFrame()
        right.setObjectName("sidePanel")
        self.side_panel = right
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(12, 12, 12, 12)
        right_layout.setSpacing(10)
        self.pose_card = MetricCard("位姿", "--")
        self.sensor_card = MetricCard("传感器", "--")
        self.ultra_card = MetricCard("超声波", "--")
        self.map_card = MetricCard("地图", "--")
        right_layout.addWidget(self.pose_card)
        right_layout.addWidget(self.sensor_card)
        right_layout.addWidget(self.ultra_card)
        right_layout.addWidget(self.map_card)
        goal_label = QtWidgets.QLabel("目标点")
        goal_label.setObjectName("sectionTitle")
        right_layout.addWidget(goal_label)
        self.goal_list = QtWidgets.QListWidget()
        self.goal_list.setMinimumWidth(280)
        right_layout.addWidget(self.goal_list, 1)
        self.navigate_btn = QtWidgets.QPushButton("导航")
        self.navigate_btn.clicked.connect(self.navigate_selected_goal)
        right_layout.addWidget(self.navigate_btn)
        body.addWidget(right)
        outer.addLayout(body, 1)

        self.setCentralWidget(root)
        self._build_settings()

    def _build_settings(self):
        self.settings_open = False
        self.settings_panel = QtWidgets.QFrame(self.root_widget)
        self.settings_panel.setObjectName("settingsPanel")
        self.settings_panel.setMinimumWidth(430)
        self.settings_panel.hide()
        shadow = QtWidgets.QGraphicsDropShadowEffect(self.settings_panel)
        shadow.setBlurRadius(24)
        shadow.setOffset(-8, 0)
        shadow.setColor(QtGui.QColor(16, 24, 40, 46))
        self.settings_panel.setGraphicsEffect(shadow)
        panel_layout = QtWidgets.QVBoxLayout(self.settings_panel)
        panel_layout.setContentsMargins(12, 12, 12, 12)
        panel_layout.setSpacing(10)

        title_bar = QtWidgets.QFrame()
        title_bar.setObjectName("settingsTitleBar")
        title_layout = QtWidgets.QHBoxLayout(title_bar)
        title_layout.setContentsMargins(14, 8, 8, 8)
        title_layout.setSpacing(10)
        title = QtWidgets.QLabel("设置")
        title.setObjectName("settingsTitle")
        close = QtWidgets.QPushButton("×")
        close.setObjectName("closeSettingsButton")
        close.setFixedSize(52, 52)
        close.clicked.connect(self.hide_settings)
        title_layout.addWidget(title)
        title_layout.addStretch(1)
        title_layout.addWidget(close)
        tabs = QtWidgets.QTabWidget()
        tabs.setObjectName("settingsTabs")
        tabs.addTab(self._mapping_tab(), "建图")
        tabs.addTab(self._goal_tab(), "目标点")
        tabs.addTab(self._system_tab(), "系统")
        panel_layout.addWidget(title_bar)
        panel_layout.addWidget(tabs, 1)

        self.settings_anim = QtCore.QPropertyAnimation(self.settings_panel, b"geometry", self)
        self.settings_anim.setDuration(240)
        self.settings_anim.setEasingCurve(QtCore.QEasingCurve.OutCubic)
        QtCore.QTimer.singleShot(0, self._place_settings_closed)

    def _mapping_tab(self):
        page = QtWidgets.QWidget()
        page.setObjectName("settingsPage")
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        self.mapping_name = QtWidgets.QLineEdit()
        self.mapping_name.setPlaceholderText("home_1f")
        self.mapping_state = StatusPill("IDLE")
        self.mapping_reason = QtWidgets.QLabel("等待建图")
        self.mapping_reason.setWordWrap(True)
        self.mapping_progress = QtWidgets.QProgressBar()
        self.mapping_progress.setRange(0, 100)
        self.mapping_progress.setFormat("保存进度 %p%")
        self.mapping_checks_label = QtWidgets.QLabel("RTAB-Map 3D 设备检查")
        self.mapping_checks_label.setObjectName("mappingSubTitle")
        self.preflight_list = QtWidgets.QListWidget()
        self.preflight_list.setObjectName("mappingLogList")
        self.preflight_list.setUniformItemSizes(True)
        self.preflight_list.setSpacing(0)
        self.preflight_list.setVerticalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        self.preflight_list.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.preflight_list.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        self.preflight_list.setMinimumHeight(180)
        QtWidgets.QScroller.grabGesture(
            self.preflight_list.viewport(), QtWidgets.QScroller.LeftMouseButtonGesture
        )
        QtWidgets.QScroller.grabGesture(
            self.preflight_list.viewport(), QtWidgets.QScroller.TouchGesture
        )
        start = QtWidgets.QPushButton("开始3D建图")
        finish = QtWidgets.QPushButton("结束并保存")
        cancel = QtWidgets.QPushButton("取消建图")
        cancel.setObjectName("outlineDangerButton")
        for button in (start, finish, cancel):
            button.setProperty("mappingAction", True)
            button.setMinimumHeight(34)
            button.setMaximumHeight(36)
            button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        start.clicked.connect(self.start_mapping)
        finish.clicked.connect(self.finish_mapping)
        cancel.clicked.connect(self.cancel_mapping)
        actions = QtWidgets.QFrame()
        actions.setObjectName("mappingActions")
        actions_layout = QtWidgets.QHBoxLayout(actions)
        actions_layout.setContentsMargins(0, 0, 0, 0)
        actions_layout.setSpacing(8)
        actions_layout.addWidget(start)
        actions_layout.addWidget(finish)
        actions_layout.addWidget(cancel)
        layout.addWidget(QtWidgets.QLabel("地图名称"))
        layout.addWidget(self.mapping_name)
        layout.addWidget(self.mapping_state)
        layout.addWidget(self.mapping_progress)
        layout.addWidget(self.mapping_reason)
        layout.addWidget(self.mapping_checks_label)
        layout.addWidget(self.preflight_list, 1)
        layout.addWidget(actions)
        return page

    def _goal_tab(self):
        page = QtWidgets.QWidget()
        page.setObjectName("settingsPage")
        layout = QtWidgets.QGridLayout(page)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(12)
        layout.setColumnMinimumWidth(0, 70)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        self.goal_name = QtWidgets.QLineEdit()
        self.goal_x = QtWidgets.QDoubleSpinBox()
        self.goal_y = QtWidgets.QDoubleSpinBox()
        self.goal_yaw = QtWidgets.QDoubleSpinBox()
        for spin in (self.goal_x, self.goal_y):
            spin.setRange(-1000.0, 1000.0)
            spin.setDecimals(3)
        self.goal_yaw.setRange(-6.283, 6.283)
        self.goal_yaw.setDecimals(3)
        save = QtWidgets.QPushButton("保存目标点")
        delete = QtWidgets.QPushButton("删除选中目标")
        delete.setObjectName("outlineDangerButton")
        init_pose = QtWidgets.QPushButton("设置初始位姿(点地图填X/Y)")
        nav_point = QtWidgets.QPushButton("导航到此点")
        for button in (save, delete, init_pose, nav_point):
            button.setMinimumHeight(50)
        save.clicked.connect(self.save_goal)
        delete.clicked.connect(self.delete_selected_goal)
        init_pose.clicked.connect(self.set_initial_pose)
        nav_point.clicked.connect(self.navigate_to_point)

        def add_goal_row(row: int, text: str, widget: QtWidgets.QWidget):
            label = QtWidgets.QLabel(text)
            label.setObjectName("goalFormLabel")
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setMinimumHeight(42)
            layout.addWidget(label, row, 0)
            layout.addWidget(widget, row, 1)

        add_goal_row(0, "名称", self.goal_name)
        add_goal_row(1, "X", self.goal_x)
        add_goal_row(2, "Y", self.goal_y)
        add_goal_row(3, "Yaw", self.goal_yaw)
        layout.addWidget(init_pose, 4, 0, 1, 2)
        layout.addWidget(nav_point, 5, 0, 1, 2)
        layout.addWidget(save, 6, 0, 1, 2)
        layout.addWidget(delete, 7, 0, 1, 2)
        layout.setRowStretch(8, 1)
        return page

    def _system_tab(self):
        page = QtWidgets.QWidget()
        page.setObjectName("settingsPage")
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)
        self.hardware_label = QtWidgets.QLabel("--")
        self.runtime_label = QtWidgets.QLabel("运行：未启动")
        self.localization_label = QtWidgets.QLabel("--")
        self.passability_label = QtWidgets.QLabel("--")
        self.mapping_log_label = QtWidgets.QLabel("建图日志：--")
        self.mapping_quality_label = QtWidgets.QLabel("地图质量：--")
        self.mapping_version_label = QtWidgets.QLabel("地图版本：--")
        for label in (
            self.runtime_label,
            self.hardware_label,
            self.localization_label,
            self.passability_label,
            self.mapping_log_label,
            self.mapping_quality_label,
            self.mapping_version_label,
        ):
            label.setWordWrap(True)
            layout.addWidget(label)
        version_title = QtWidgets.QLabel("最近地图版本")
        version_title.setObjectName("mappingSubTitle")
        self.map_version_combo = QtWidgets.QComboBox()
        self.map_version_combo.setMinimumHeight(42)
        self.activate_version_btn = QtWidgets.QPushButton("设为当前版本")
        self.activate_version_btn.setMinimumHeight(42)
        self.activate_version_btn.clicked.connect(self.activate_selected_map_version)
        layout.addWidget(version_title)
        layout.addWidget(self.map_version_combo)
        layout.addWidget(self.activate_version_btn)
        layout.addStretch(1)
        return page

    def _apply_styles(self):
        self.setStyleSheet(
            f"""
            QMainWindow, QWidget {{
                background: {COLORS['bg']};
                color: {COLORS['ink']};
                font-family: Sans;
                font-size: 14px;
            }}
            #appTitle {{
                font-size: 26px;
                font-weight: 800;
                color: {COLORS['dark']};
            }}
            QPushButton {{
                min-height: 44px;
                border-radius: 10px;
                border: 1px solid {COLORS['blue']};
                background: {COLORS['blue']};
                color: #ffffff;
                font-weight: 700;
                font-size: 15px;
                padding: 8px 14px;
            }}
            QPushButton:hover {{ background: #1f6fd1; }}
            QPushButton:pressed {{ background: #155fb6; }}
            #runButton[runtimeState="idle"] {{
                background: {COLORS['green']};
                border-color: {COLORS['green']};
                min-width: 88px;
            }}
            #runButton[runtimeState="starting"] {{
                background: {COLORS['amber']};
                border-color: {COLORS['amber']};
                min-width: 88px;
            }}
            #runButton[runtimeState="stopping"] {{
                background: {COLORS['amber']};
                border-color: {COLORS['amber']};
                min-width: 88px;
            }}
            #runButton[runtimeState="running"] {{
                background: #ffffff;
                color: {COLORS['green']};
                border-color: {COLORS['green']};
                min-width: 88px;
            }}
            #runButton[runtimeState="error"] {{
                background: #ffffff;
                color: {COLORS['red']};
                border-color: {COLORS['red']};
                min-width: 88px;
            }}
            QPushButton[mappingAction="true"] {{
                min-height: 34px;
                max-height: 36px;
                border-radius: 9px;
                padding: 2px 10px;
                font-size: 13px;
            }}
            #dangerButton {{
                background: {COLORS['red']};
                border-color: {COLORS['red']};
            }}
            #outlineDangerButton {{
                background: #ffffff;
                color: {COLORS['red']};
                border-color: {COLORS['red']};
            }}
            #outlineDangerButton:hover {{
                background: rgba(192, 57, 43, 0.10);
            }}
            #sidePanel, #metricCard, #settingsPage {{
                background: {COLORS['panel']};
            }}
            #sidePanel, #metricCard, #settingsPage {{
                border: 1px solid {COLORS['line']};
                border-radius: 14px;
            }}
            #mappingActions {{
                background: transparent;
                border: none;
            }}
            #metricCard {{
                min-height: 72px;
            }}
            #metricTitle {{
                color: {COLORS['muted']};
                font-weight: 700;
                font-size: 12px;
            }}
            #metricValue {{
                color: {COLORS['ink']};
                font-weight: 750;
                font-size: 15px;
            }}
            #sectionTitle {{
                color: {COLORS['dark']};
                font-weight: 800;
                font-size: 16px;
            }}
            QListWidget, QLineEdit, QDoubleSpinBox, QComboBox {{
                border: 1px solid {COLORS['line']};
                border-radius: 10px;
                background: #ffffff;
                min-height: 42px;
                padding: 4px 8px;
                font-size: 15px;
            }}
            QComboBox::drop-down {{
                border: none;
                width: 34px;
            }}
            QListWidget::item {{
                min-height: 40px;
                padding: 6px;
            }}
            QListWidget#mappingLogList {{
                background: #f8fafc;
                min-height: 180px;
                padding: 4px 6px;
            }}
            QListWidget#mappingLogList::item {{
                min-height: 24px;
                padding: 2px 4px;
                color: {COLORS['dark']};
            }}
            #mappingSubTitle {{
                color: {COLORS['dark']};
                background: transparent;
                font-size: 13px;
                font-weight: 800;
                padding: 0px;
            }}
            QListWidget#mappingLogList QScrollBar:vertical {{
                border: none;
                background: #edf2f7;
                width: 16px;
                margin: 3px 2px 3px 2px;
                border-radius: 8px;
            }}
            QListWidget#mappingLogList QScrollBar::handle:vertical {{
                background: #aebbd0;
                border-radius: 6px;
                min-height: 36px;
            }}
            QListWidget#mappingLogList QScrollBar::handle:vertical:hover {{
                background: #8fa0b8;
            }}
            QListWidget#mappingLogList QScrollBar::add-line:vertical,
            QListWidget#mappingLogList QScrollBar::sub-line:vertical {{
                height: 0px;
                width: 0px;
                border: none;
                background: transparent;
            }}
            QListWidget#mappingLogList QScrollBar::add-page:vertical,
            QListWidget#mappingLogList QScrollBar::sub-page:vertical {{
                background: transparent;
            }}
            QListWidget#mappingLogList QScrollBar:horizontal {{
                height: 0px;
                background: transparent;
            }}
            QLabel#goalFormLabel {{
                color: #000000;
                background: transparent;
                border: none;
                font-weight: 750;
                padding: 0px;
            }}
            QProgressBar {{
                border: 1px solid {COLORS['line']};
                border-radius: 10px;
                min-height: 24px;
                background: #e5ebf2;
                text-align: center;
            }}
            QProgressBar::chunk {{
                border-radius: 10px;
                background: {COLORS['green']};
            }}
            #settingsPanel {{
                border: 1px solid {COLORS['line']};
                border-radius: 16px;
                background: {COLORS['panel']};
            }}
            #settingsTitleBar {{
                border: 1px solid {COLORS['line']};
                border-radius: 16px;
                background: {COLORS['panel']};
            }}
            #settingsTitle {{
                color: {COLORS['dark']};
                font-size: 20px;
                font-weight: 800;
                background: transparent;
            }}
            #closeSettingsButton {{
                min-width: 52px;
                min-height: 52px;
                border-radius: 16px;
                border: 1px solid #cbd5e1;
                background: #ffffff;
                color: {COLORS['dark']};
                font-size: 30px;
                font-weight: 800;
                padding: 0;
            }}
            #closeSettingsButton:hover {{
                background: rgba(192, 57, 43, 0.10);
                color: {COLORS['red']};
                border-color: rgba(192, 57, 43, 0.55);
            }}
            #settingsTabs {{
                background: {COLORS['panel']};
            }}
            #settingsTabs::pane {{
                border: 1px solid {COLORS['line']};
                border-radius: 14px;
                background: {COLORS['panel']};
                top: -1px;
            }}
            QTabBar::tab {{
                min-height: 34px;
                max-height: 36px;
                min-width: 96px;
                padding: 4px 12px;
                margin-right: 6px;
                border: 1px solid {COLORS['line']};
                border-radius: 10px;
                background: #f8fafc;
                color: {COLORS['dark']};
                font-weight: 750;
            }}
            QTabBar::tab:selected {{
                background: {COLORS['blue']};
                color: #ffffff;
                border-color: {COLORS['blue']};
            }}
            QTabBar::tab:hover {{
                border-color: {COLORS['blue']};
            }}
            """
        )

    def _ros_source_command(self, command: str) -> str:
        log_dir = self.workspace_root / ".ros" / "log"
        return (
            "set +u; "
            "source /opt/ros/humble/setup.bash; "
            "source install/setup.bash; "
            'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${SMARTWHEEL_ROS_DOMAIN_ID:-0}}"; '
            f'export ROS_LOG_DIR="${{ROS_LOG_DIR:-{log_dir}}}"; '
            'mkdir -p "$ROS_LOG_DIR"; '
            f"{command}"
        )

    def _set_run_state(self, state: str, text: str, detail: str = ""):
        self.run_btn.setText(text)
        self.run_btn.setProperty("runtimeState", state)
        self.run_btn.style().unpolish(self.run_btn)
        self.run_btn.style().polish(self.run_btn)
        self.run_btn.update()
        if hasattr(self, "runtime_label"):
            suffix = f" - {detail}" if detail else ""
            self.runtime_label.setText(f"运行：{text}{suffix}")

    def _system_process_running(self) -> bool:
        return (
            self.system_process is not None
            and self.system_process.state() != QtCore.QProcess.NotRunning
        )

    def _smartwheel_service_active(self) -> bool:
        """True iff the systemd user unit smartwheel.service is currently running.

        We check this so the toggle button can route through ``systemctl --user
        stop`` when the unit is in charge. Sending SIGINT to individual ROS
        nodes does not work well when systemd owns the process tree because
        ``Restart=on-failure`` will resurrect them.
        """
        try:
            result = subprocess.run(
                ["systemctl", "--user", "is-active", "smartwheel.service"],
                capture_output=True,
                text=True,
                timeout=4,
            )
        except Exception:
            return False
        return result.stdout.strip() == "active"

    def _smartwheel_service_unit_present(self) -> bool:
        try:
            result = subprocess.run(
                ["systemctl", "--user", "list-unit-files", "smartwheel.service"],
                capture_output=True,
                text=True,
                timeout=4,
            )
        except Exception:
            return False
        if result.returncode != 0:
            return False
        return any(
            line.split()[0] == "smartwheel.service"
            for line in result.stdout.splitlines()
            if line.strip()
        )

    def _systemctl_user(self, action: str, timeout: int = 25) -> bool:
        try:
            result = subprocess.run(
                ["systemctl", "--user", action, "smartwheel.service"],
                capture_output=True,
                text=True,
                timeout=timeout,
            )
            return result.returncode == 0
        except Exception:
            return False

    @staticmethod
    def _use_systemd_backend() -> bool:
        return os.environ.get("SMARTWHEEL_GUI_USE_SYSTEMD", "false").strip().lower() in {
            "1",
            "true",
            "yes",
            "on",
        }

    def _status_indicates_system_active(self, status: Dict) -> bool:
        sensors = status.get("sensors") or {}
        if any(
            bool((sensors.get(key) or {}).get("online"))
            for key in ("scan", "laser", "imu", "odom", "base")
        ):
            return True
        if any(
            bool((value or {}).get("online"))
            for key, value in sensors.items()
            if str(key).startswith("camera_")
        ):
            return True
        ultrasonic = sensors.get("ultrasonic") or []
        if any(bool(item.get("online")) for item in ultrasonic):
            return True
        if status.get("safety_state") not in (None, "", "UNKNOWN"):
            return True
        return status.get("navigation_status") not in (None, "", "IDLE", "UNKNOWN")

    def _ros_nodes_running(self) -> bool:
        process = QtCore.QProcess(self)
        process.setWorkingDirectory(str(self.workspace_root))
        process.start("/bin/bash", ["-lc", self._ros_source_command("ros2 node list --no-daemon")])
        if not process.waitForFinished(2500):
            process.kill()
            process.waitForFinished(1000)
            return False
        output = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace")
        return any(line.strip() in KNOWN_BACKEND_NODES for line in output.splitlines())

    def _managed_backend_pids(self) -> list[int]:
        try:
            result = subprocess.run(
                ["ps", "-eo", "pid=,cmd="],
                cwd=str(self.workspace_root),
                capture_output=True,
                text=True,
                timeout=2,
            )
        except Exception:
            return []
        if result.returncode != 0:
            return []
        current_pid = os.getpid()
        pids = []
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            pid_text, _, command = stripped.partition(" ")
            try:
                pid = int(pid_text)
            except ValueError:
                continue
            if pid == current_pid or "wheelchair_native_gui" in command:
                continue
            if any(pattern in command for pattern in MANAGED_BACKEND_PATTERNS):
                pids.append(pid)
        return sorted(set(pids))

    @staticmethod
    def _signal_pids(pids: list[int], sig: signal.Signals):
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                continue
            except PermissionError:
                continue

    def _wait_for_backend_exit(self, timeout_ms: int) -> bool:
        timer = QtCore.QElapsedTimer()
        timer.start()
        while timer.elapsed() < timeout_ms:
            QtWidgets.QApplication.processEvents(
                QtCore.QEventLoop.ExcludeUserInputEvents, 50
            )
            if not self._managed_backend_pids():
                return True
            QtCore.QThread.msleep(100)
        return not self._managed_backend_pids()

    def _signal_system_group(self, sig: signal.Signals):
        pgid = self.system_pgid
        if pgid is None and self._system_process_running():
            try:
                pgid = os.getpgid(int(self.system_process.processId()))
            except Exception:
                pgid = None
        if pgid is None:
            return
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            pass
        except PermissionError:
            pass

    def _run_hardware_shutdown_script(self):
        script = self.workspace_root / "scripts" / "hardware_shutdown.sh"
        if not script.exists():
            return
        try:
            subprocess.run(
                [str(script), "--no-source", "--quiet"],
                cwd=str(self.workspace_root),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=4,
            )
        except Exception:
            pass

    def start_system(self):
        runtime_state = self.run_btn.property("runtimeState")
        # Toggle: if anything looks like the system is already running, stop it.
        # Detection covers our own QProcess, any matching backend PIDs found via
        # ps, ROS nodes visible on the graph, or active sensor topics.
        service_active = self._smartwheel_service_active()
        if (
            runtime_state in ("starting", "running", "stopping")
            or self._system_process_running()
            or service_active
            or self._status_indicates_system_active(self.latest_status)
            or self._ros_nodes_running()
        ):
            self.stop_system_process()
            # Mark this as an explicit user-initiated stop so the periodic
            # status refresh does not flip the button back to "running" on
            # the next tick due to cached topic activity.
            self._last_explicit_stop_at = time.monotonic()
            self._set_run_state("idle", "运行", "ROS2 后端已关闭")
            self.statusBar().showMessage("ROS2 系统已关闭", 4000)
            return

        # Native GUI is an interactive mapping/navigation session, so it starts
        # full_system directly with radar enabled. The user unit remains an
        # opt-in boot/background path via SMARTWHEEL_GUI_USE_SYSTEMD=true.
        if self._use_systemd_backend() and self._smartwheel_service_unit_present():
            self._set_run_state("starting", "启动中", "通过 systemd 启动 smartwheel.service")
            self.system_launch_detail = "smartwheel.service (full_system.launch.py)"
            ok = self._systemctl_user("start", timeout=15)
            if not ok:
                self._set_run_state("error", "运行失败", "systemctl start 失败")
                return
            # systemd takes a few seconds to bring up nodes; show a transient
            # "starting" state and let the periodic status refresh flip to
            # "running" once the backend is reachable.
            self._set_run_state("running", "运行中", self.system_launch_detail)
            self.statusBar().showMessage("ROS2 后端启动中 (systemd 管理)", 6000)
            return

        if not (self.workspace_root / "install" / "setup.bash").exists():
            self._set_run_state("error", "运行失败", "缺少 install/setup.bash")
            QtWidgets.QMessageBox.critical(
                self,
                "无法启动",
                "没有找到 install/setup.bash，请先完成 colcon build。",
            )
            return

        launch_args = [
            "enable_web_ui:=false",
            "enable_native_gui:=false",
            "enable_xtm60_radar:=true",
            "enable_dual_xtm60:=true",
        ]
        map_path = self.mapping.last_map_yaml
        if map_path and Path(map_path).exists():
            launch_args.append(f"map:={map_path}")
            self.system_launch_detail = f"full_system.launch.py，双雷达导航，地图：{map_path.name}"
        else:
            self.system_launch_detail = "full_system.launch.py，双雷达导航，默认空地图"
        launch = (
            "exec ros2 launch wheelchair_bringup full_system.launch.py "
            + " ".join(shlex.quote(arg) for arg in launch_args)
        )
        self.system_process = QtCore.QProcess(self)
        self.system_process.setWorkingDirectory(str(self.workspace_root))
        self.system_process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.system_process.readyReadStandardOutput.connect(self._handle_system_output)
        self.system_process.started.connect(self._on_system_started)
        self.system_process.finished.connect(self._on_system_finished)
        self.system_process.errorOccurred.connect(self._on_system_error)
        self._set_run_state("starting", "启动中", "正在启动 ROS2 后端")
        self.system_process.start(
            "/usr/bin/setsid",
            ["/bin/bash", "-lc", self._ros_source_command(launch)],
        )

    def _on_system_started(self):
        if self.system_process is not None:
            try:
                self.system_pgid = int(self.system_process.processId())
            except Exception:
                self.system_pgid = None
        self._set_run_state("running", "运行中", self.system_launch_detail or "full_system.launch.py")

    def _handle_system_output(self):
        if self.system_process is None:
            return
        output = bytes(self.system_process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        lines = [line.strip() for line in output.splitlines() if line.strip()]
        if not lines:
            return
        self.system_log_tail = (self.system_log_tail + lines)[-20:]
        self.statusBar().showMessage(lines[-1], 6000)

    def _on_system_error(self, _error):
        if self.closing:
            return
        self.system_pgid = None
        self._set_run_state("error", "运行失败", "QProcess 启动失败")

    def _on_system_finished(self, exit_code: int, _exit_status):
        if self.closing:
            return
        backend_pids = self._managed_backend_pids()
        if backend_pids:
            self._set_run_state("running", "已运行", "后端节点仍在线")
        else:
            self.system_pgid = None
            if exit_code == 0:
                self._set_run_state("idle", "运行", "ROS2 后端已退出")
            else:
                detail = self.system_log_tail[-1] if self.system_log_tail else f"退出码 {exit_code}"
                self._set_run_state("error", "运行失败", detail)
        self.system_process = None

    def stop_system_process(self):
        self._set_run_state("stopping", "关闭中", "正在发布急停和零速")
        try:
            self.bridge.node.request_hardware_shutdown()
        except Exception:
            pass

        # If systemd is in charge of the backend, stop the unit cleanly.
        # Sending SIGINT to individual ROS nodes will fight with
        # ``Restart=on-failure`` and the backend would come right back.
        used_systemctl = False
        if self._smartwheel_service_active():
            used_systemctl = self._systemctl_user("stop", timeout=20)

        # Always also try to wind down anything we launched directly via
        # QProcess and any leftover backend PIDs (e.g. mapping launch the
        # mapping_manager kicked off, or stale processes from a crashed
        # service). When systemctl already handled the unit, _managed_backend_pids
        # should now be empty so these are no-ops.
        self._signal_system_group(signal.SIGINT)
        self._signal_pids(self._managed_backend_pids(), signal.SIGINT)
        process = self.system_process
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            process.terminate()
        if not self._wait_for_backend_exit(5000):
            self._signal_system_group(signal.SIGTERM)
            self._signal_pids(self._managed_backend_pids(), signal.SIGTERM)
            self._wait_for_backend_exit(2500)
        process = self.system_process
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            if not process.waitForFinished(500):
                process.kill()
                process.waitForFinished(1000)
        self.system_process = None
        self.system_pgid = None
        self._run_hardware_shutdown_script()
        if used_systemctl:
            self.statusBar().showMessage("ROS2 已通过 systemd 停止", 4000)

    def shutdown_hardware(self):
        try:
            self.bridge.node.request_hardware_shutdown()
        finally:
            self.stop_system_process()
            self._set_run_state("idle", "运行", "硬件已关闭")

    def toggle_settings(self):
        if self.settings_open:
            self.hide_settings()
        else:
            self.show_settings()

    def _settings_geometry(self, opened: bool) -> QtCore.QRect:
        root_rect = self.root_widget.rect()
        side_rect = self.side_panel.geometry()
        width = min(max(430, side_rect.width() + 48), max(360, root_rect.width() - 28))
        y = side_rect.y()
        if hasattr(self, "shutdown_btn"):
            y = min(y, self.shutdown_btn.mapTo(self.root_widget, QtCore.QPoint(0, 0)).y() - 6)
        y = max(0, y)
        height = max(300, root_rect.height() - y - 14)
        open_x = root_rect.width() - width - 14
        closed_x = root_rect.width() + 8
        return QtCore.QRect(open_x if opened else closed_x, y, width, height)

    def _place_settings_closed(self):
        if not hasattr(self, "settings_panel"):
            return
        self.settings_panel.setGeometry(self._settings_geometry(False))
        self.settings_panel.hide()

    def show_settings(self):
        self.settings_anim.stop()
        self.settings_panel.setGeometry(self._settings_geometry(False))
        self.settings_panel.show()
        self.settings_panel.raise_()
        self.settings_anim.setStartValue(self._settings_geometry(False))
        self.settings_anim.setEndValue(self._settings_geometry(True))
        self.settings_anim.start()
        self.settings_open = True

    def hide_settings(self):
        self.settings_anim.stop()
        self.settings_anim.setStartValue(self.settings_panel.geometry())
        self.settings_anim.setEndValue(self._settings_geometry(False))
        try:
            self.settings_anim.finished.disconnect()
        except TypeError:
            pass
        self.settings_anim.finished.connect(self._hide_settings_after_animation)
        self.settings_anim.start()
        self.settings_open = False

    def _hide_settings_after_animation(self):
        if not self.settings_open:
            self.settings_panel.hide()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not hasattr(self, "settings_panel"):
            return
        if self.settings_open:
            self.settings_panel.setGeometry(self._settings_geometry(True))
            self.settings_panel.raise_()
        else:
            self.settings_panel.setGeometry(self._settings_geometry(False))

    def refresh_all(self):
        try:
            self.latest_status = self.bridge.node.status()
            self.latest_goals = self.bridge.node.list_goals()
            self.latest_map = self.bridge.node.map_snapshot() or self.mapping.map_snapshot() or self.latest_map
            semantic = self.bridge.node.semantic_map()
            mapping_status = self.mapping.status(self.latest_status)
        except Exception as exc:
            self.statusBar().showMessage(str(exc), 4000)
            return

        if self.latest_map:
            self.map_canvas.set_map(self.latest_map)
        self.map_canvas.set_goals(self.latest_goals)
        self.map_canvas.set_semantic(semantic)
        self.map_canvas.set_pose(self.latest_status.get("pose", {}))
        self._update_status_cards()
        self._update_goals()
        self._update_mapping(mapping_status)

    def _update_status_cards(self):
        status = self.latest_status
        # The run button is purely a manual toggle. Periodic refresh used to
        # auto-flip it between "运行" and "已运行" based on whether topics
        # appeared online, which created a flicker because cached status
        # snapshots transition between active/inactive while the backend is
        # spinning up or shutting down. The button's visible state is now
        # only changed by explicit user actions (start_system,
        # stop_system_process) and by the initial detection at startup.
        # Status info still drives every other card below.
        self.run_btn.setToolTip(f"安全状态：{status.get('safety_state', 'UNKNOWN')}")
        self.nav_pill.set_state(status.get("navigation_status", "IDLE"))
        pose = status.get("pose", {})
        self.pose_card.set_value(
            f"{fmt_float(pose.get('x'))}, {fmt_float(pose.get('y'))}, {fmt_float(pose.get('yaw'))}"
        )
        sensors = status.get("sensors", {})
        online = []
        camera_online = any(
            bool((value or {}).get("online"))
            for key, value in sensors.items()
            if str(key).startswith("camera_")
        )
        for key, label in (("scan", "scan"), ("imu", "imu"), ("odom", "odom")):
            if (sensors.get(key) or {}).get("online"):
                online.append(label)
        if camera_online:
            online.append("cam")
        self.sensor_card.set_value(" / ".join(online) if online else "--")
        ultrasonic = sensors.get("ultrasonic") or []
        if ultrasonic:
            item = ultrasonic[0]
            mm = item.get("range_mm")
            if mm is None and item.get("range_m") is not None:
                mm = round(float(item["range_m"]) * 1000.0)
            self.ultra_card.set_value(f"{mm} mm" if mm is not None else "--")
        else:
            self.ultra_card.set_value("--")
        if self.latest_map:
            width = float(self.latest_map.get("width", 0)) * float(self.latest_map.get("resolution", 0))
            height = float(self.latest_map.get("height", 0)) * float(self.latest_map.get("resolution", 0))
            self.map_card.set_value(f"{fmt_float(width, 1)}m x {fmt_float(height, 1)}m")
        hardware = status.get("hardware_status", {})
        localization = status.get("localization_health", {})
        passability = status.get("passability_status", {})
        self.hardware_label.setText(f"硬件：{hardware.get('state', 'UNKNOWN')} - {hardware.get('reason', '')}")
        self.localization_label.setText(
            f"定位：{localization.get('state', 'UNKNOWN')} - {localization.get('reason', '')}"
        )
        self.passability_label.setText(
            f"通行：{passability.get('state', 'UNKNOWN')} - {passability.get('reason', '')}"
        )

    def _update_goals(self):
        selected = self.goal_list.currentItem().data(QtCore.Qt.UserRole) if self.goal_list.currentItem() else None
        self.goal_list.blockSignals(True)
        self.goal_list.clear()
        for key, goal in self.latest_goals.items():
            position = goal.get("position", [0.0, 0.0])
            item = QtWidgets.QListWidgetItem(
                f"{goal.get('label', key)}   {fmt_float(position[0])}, {fmt_float(position[1])}"
            )
            item.setData(QtCore.Qt.UserRole, key)
            self.goal_list.addItem(item)
            if key == selected:
                self.goal_list.setCurrentItem(item)
        self.goal_list.blockSignals(False)

    def _update_mapping(self, status: Dict):
        self.mapping_state.set_state(status.get("state", "IDLE"))
        self.mapping_reason.setText(status.get("reason", "等待建图"))
        self.mapping_progress.setValue(int(status.get("progress", 0)))
        state = status.get("state", "IDLE")
        if state == "MAPPING":
            self.mapping_progress.setFormat("保存进度 0%")
        elif state == "SAVING":
            self.mapping_progress.setFormat("保存中 %p%")
        elif state == "MAP_READY":
            self.mapping_progress.setFormat("保存完成 %p%")
        elif state == "ERROR":
            self.mapping_progress.setFormat("保存失败")
        else:
            self.mapping_progress.setFormat("保存进度 %p%")
        if hasattr(self, "mapping_log_label"):
            log_path = status.get("log_path")
            self.mapping_log_label.setText(f"建图日志：{log_path or '--'}")
        if hasattr(self, "mapping_quality_label"):
            quality = status.get("quality_status") or {}
            if quality:
                reasons = quality.get("reasons") or []
                message = reasons[0].get("message", "") if reasons else ""
                report_path = status.get("quality_report")
                report_text = f"；报告：{report_path}" if report_path else ""
                self.mapping_quality_label.setText(
                    f"地图质量：{quality.get('verdict', '--')} ({quality.get('score', '--')}) {message}{report_text}"
                )
            else:
                self.mapping_quality_label.setText("地图质量：--")
        if hasattr(self, "mapping_version_label"):
            version = status.get("map_version") or {}
            if version:
                self.mapping_version_label.setText(
                    f"地图版本：{version.get('version_id', '--')}；当前：{version.get('current_yaml', '--')}"
                )
            else:
                self.mapping_version_label.setText("地图版本：--")
        if hasattr(self, "map_version_combo"):
            selected = self.map_version_combo.currentData()
            current_version = (status.get("map_version") or {}).get("version_id")
            target = selected or current_version
            versions = status.get("recent_map_versions") or []
            self.map_version_combo.blockSignals(True)
            self.map_version_combo.clear()
            for version in versions:
                version_id = version.get("version_id")
                if not version_id:
                    continue
                quality = version.get("quality") or {}
                verdict = quality.get("verdict", "--")
                label = f"{version_id}  {verdict}"
                self.map_version_combo.addItem(label, version_id)
                if version_id == target:
                    self.map_version_combo.setCurrentIndex(self.map_version_combo.count() - 1)
            self.map_version_combo.blockSignals(False)
            self.activate_version_btn.setEnabled(self.map_version_combo.count() > 0)
        self.preflight_list.clear()
        for check in status.get("preflight", {}).get("checks", []):
            mark = "OK" if check.get("ok") else ("REQ" if check.get("required") else "WARN")
            item = QtWidgets.QListWidgetItem(f"{mark}  {check.get('label')}")
            item.setToolTip(check.get("detail", ""))
            self.preflight_list.addItem(item)

    def selected_goal_key(self) -> Optional[str]:
        item = self.goal_list.currentItem()
        return item.data(QtCore.Qt.UserRole) if item else None

    def _nav_precondition_ok(self) -> bool:
        if not (self._system_process_running() or self._status_indicates_system_active(self.latest_status)):
            QtWidgets.QMessageBox.warning(self, "导航后端未运行", "请先点击\"运行\"启动导航后端，再发送目标。")
            return False
        loc = self.latest_status.get("localization_health") or {}
        if not (bool(loc.get("healthy")) or loc.get("state") == "GOOD"):
            QtWidgets.QMessageBox.warning(
                self, "定位未就绪",
                f"定位状态：{loc.get('state', 'UNKNOWN')}。\n"
                "请先在地图上点轮椅当前位置→设好 Yaw→点\"设置初始位姿\"，待定位变 GOOD 再导航。",
            )
            return False
        return True

    def navigate_selected_goal(self):
        key = self.selected_goal_key()
        if not key:
            return
        if not self._nav_precondition_ok():
            return
        self.bridge.node.send_named_goal(key)

    def navigate_to_point(self):
        if not self._nav_precondition_ok():
            return
        self.bridge.node.send_goal_pose(self.goal_x.value(), self.goal_y.value(), self.goal_yaw.value())
        self.statusBar().showMessage(
            f"已发送目标 x={self.goal_x.value():.2f} y={self.goal_y.value():.2f}，"
            "若 Nav2 aborted 多为目标在墙里/不可达，请点更空旷的点", 6000
        )

    def set_initial_pose(self):
        self.bridge.node.set_initial_pose(
            self.goal_x.value(), self.goal_y.value(), self.goal_yaw.value()
        )
        self.statusBar().showMessage(
            f"已发送初始位姿 x={self.goal_x.value():.2f} y={self.goal_y.value():.2f} "
            f"yaw={self.goal_yaw.value():.2f}，等待 AMCL 收敛后再导航", 6000
        )

    def save_goal(self):
        name = self.goal_name.text().strip()
        if not name:
            return
        self.bridge.node.add_goal(
            {
                "name": name,
                "x": self.goal_x.value(),
                "y": self.goal_y.value(),
                "yaw": self.goal_yaw.value(),
                "frame_id": "map",
            }
        )
        self.refresh_all()

    def delete_selected_goal(self):
        key = self.selected_goal_key()
        if key:
            self.bridge.node.delete_goal(key)
            self.refresh_all()

    def fill_goal_from_map(self, x: float, y: float):
        self.goal_x.setValue(x)
        self.goal_y.setValue(y)
        self.show_settings()

    def start_mapping(self):
        node_names = self.mapping._ros_node_names()
        conflicts = sorted(CONFLICTING_NAV_NODES.intersection(node_names))
        if conflicts:
            self._set_run_state("stopping", "切换中", "正在关闭导航模式")
            self.statusBar().showMessage("检测到导航模式，正在切换到建图模式", 5000)
            self.stop_system_process()
            self._set_run_state("idle", "运行", "已切换到建图模式")
            QtWidgets.QApplication.processEvents(
                QtCore.QEventLoop.ExcludeUserInputEvents, 50
            )
        result = self.mapping.start(
            self.latest_status or self.bridge.node.status(),
            self.mapping_name.text().strip() or None,
        )
        self._update_mapping(result)

    def finish_mapping(self):
        result = self.mapping.finish(self.mapping_name.text().strip() or None)
        self._update_mapping(result)
        self.refresh_all()

    def cancel_mapping(self):
        self._update_mapping(self.mapping.cancel())

    def activate_selected_map_version(self):
        if not hasattr(self, "map_version_combo"):
            return
        version_id = self.map_version_combo.currentData()
        if not version_id:
            return
        result = self.mapping.activate_version(version_id)
        self._update_mapping(result)
        self.latest_map = self.mapping.map_snapshot() or self.latest_map
        if self.latest_map:
            self.map_canvas.set_map(self.latest_map)
        self.statusBar().showMessage(f"已切换地图版本：{version_id}", 4000)

    def closeEvent(self, event):
        self.closing = True
        try:
            self.stop_system_process()
            self.mapping.shutdown()
            self.bridge.shutdown()
        finally:
            event.accept()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--named-goals-path", default=default_named_goals_path())
    parser.add_argument("--semantic-map-path", default=default_semantic_map_path())
    args, _ = parser.parse_known_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("SmartWheel")
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal.signal(signal.SIGTERM, lambda *_args: app.quit())
    signal_timer = QtCore.QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(250)
    bridge = RosBridge(
        args.named_goals_path,
        args.semantic_map_path,
        node_name="wheelchair_native_gui_ros_bridge",
    )
    mapping = MappingManager()
    window = MainWindow(bridge, mapping)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
