#!/usr/bin/env python3
"""
science_gui.py

Operator console for the URC Science Mission.

Panels:
- Mission Clock: elapsed time since the console was opened
- Comms / Link Health: is /science_data live, how stale is it, roughly
  what rate is it arriving at
- Site Data: current site ID and GNSS coordinates
- Instrumentation: temperature, humidity, pH, life-detection signal
- Site Documentation Checklist: tracks the four things URC rule 1.b.iii
  requires per site (wide panorama w/ cardinal directions + scale,
  close-up photo w/ scale, stratigraphic profile photo, GNSS w/
  elevation/accuracy). A site can't be marked sample-ready until all
  four are logged.
- Multi-Site Comparison: table of every site visited so far, so the
  operator has an actual basis for picking which site to sample from,
  matching the "evaluate at least two sites" requirement.
- Sample Operations: state pipeline (IDLE/DRILLING/ANALYZING/COMPLETE),
  Begin Sample Collection / Abort buttons, and a 5-minute cache removal
  countdown once a sample completes (rule 1.b.vii: cache must be
  removable within 5 minutes of end of roving time).
- Command Log: running, timestamped log of operator actions and rover
  status changes.
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QTextEdit, QCheckBox, QTableWidget, QTableWidgetItem,
    QScrollArea
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor

SIGNAL_TIMEOUT_SEC = 3.0
LIFE_SIGNAL_ALERT_THRESHOLD = 0.6
CACHE_REMOVAL_WINDOW_SEC = 5 * 60  # rule 1.b.vii

CHECKLIST_ITEMS = [
    ('panorama', 'Wide panorama (cardinal directions + scale)'),
    ('closeup', 'Close-up sampling photo (with scale)'),
    ('stratigraphy', 'Stratigraphic profile photo'),
    ('gnss', 'GNSS coordinates (elevation + accuracy)'),
]


class ScienceGuiNode(Node):
    """ROS 2 half of the console: subscribing and publishing."""

    def __init__(self):
        super().__init__('science_gui_node')

        self.latest_science_data = None
        self.last_science_msg_time = None
        self.science_msg_times = []

        self.latest_status = {'state': 'IDLE', 'sample_mass_g': None}
        self.last_status_msg_time = None

        self.create_subscription(String, '/science_data', self.science_callback, 10)
        self.create_subscription(String, '/sample_status', self.status_callback, 10)
        self.command_publisher = self.create_publisher(String, '/sample_command', 10)

        self.get_logger().info('Science console node started')

    def science_callback(self, msg):
        try:
            self.latest_science_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Malformed /science_data message')
            return
        now = time.time()
        self.last_science_msg_time = now
        self.science_msg_times.append(now)
        self.science_msg_times = self.science_msg_times[-10:]

    def status_callback(self, msg):
        try:
            self.latest_status = json.loads(msg.data)
            self.last_status_msg_time = time.time()
        except json.JSONDecodeError:
            self.get_logger().warn('Malformed /sample_status message')

    def send_command(self, command):
        msg = String()
        msg.data = json.dumps({'command': command, 'timestamp': time.time()})
        self.command_publisher.publish(msg)

    def estimate_rate_hz(self):
        if len(self.science_msg_times) < 2:
            return 0.0
        span = self.science_msg_times[-1] - self.science_msg_times[0]
        if span <= 0:
            return 0.0
        return (len(self.science_msg_times) - 1) / span


class ScienceConsole(QWidget):
    def __init__(self, ros_node: ScienceGuiNode):
        super().__init__()
        self.ros_node = ros_node
        self.start_time = time.time()
        self.last_logged_state = None
        self.current_site_id = None
        self.cache_removal_deadline = None

        # site_id -> {'data': {...latest reading...}, 'checklist': {item: bool}}
        self.site_records = {}

        self.setWindowTitle('URC Science Mission — Operator Console')
        self.resize(1150, 560)
        self._build_ui()

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.ros_node, timeout_sec=0))
        self.ros_timer.start(100)

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.refresh_display)
        self.ui_timer.start(250)

    # --- UI construction ---------------------------------------------

    def _build_ui(self):
        # Everything lives inside a scrollable content widget, so on a
        # shorter screen you scroll the console instead of losing panels
        # off the bottom of the window.
        content = QWidget()
        outer = QVBoxLayout()

        self.clock_label = QLabel('Mission Elapsed: 00:00')
        self.clock_label.setFont(QFont('Arial', 16, QFont.Bold))
        outer.addWidget(self.clock_label)

        # Three columns side by side instead of stacking everything
        # vertically, so the console is wide and short like a real ops
        # dashboard instead of a long scroll.
        columns_row = QHBoxLayout()

        left_col = QVBoxLayout()
        left_col.addWidget(self._build_comms_box())
        left_col.addWidget(self._build_site_box())
        left_col.addWidget(self._build_instrument_box())
        left_col.addStretch()

        middle_col = QVBoxLayout()
        middle_col.addWidget(self._build_checklist_box())
        middle_col.addWidget(self._build_sample_box())
        middle_col.addStretch()

        right_col = QVBoxLayout()
        right_col.addWidget(self._build_comparison_table_box())
        right_col.addWidget(self._build_log_box())
        right_col.addStretch()

        columns_row.addLayout(left_col, 1)
        columns_row.addLayout(middle_col, 1)
        columns_row.addLayout(right_col, 1)

        outer.addLayout(columns_row)
        content.setLayout(outer)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)

        window_layout = QVBoxLayout()
        window_layout.setContentsMargins(0, 0, 0, 0)
        window_layout.addWidget(scroll)
        self.setLayout(window_layout)

    def _build_comms_box(self):
        box = QGroupBox('Comms / Link Health')
        layout = QVBoxLayout()
        self.link_status_label = QLabel('Status: waiting for data...')
        self.link_status_label.setFont(QFont('Arial', 10, QFont.Bold))
        self.link_rate_label = QLabel('Rate: -- Hz')
        self.link_age_label = QLabel('Last update: -- s ago')
        for w in (self.link_status_label, self.link_rate_label, self.link_age_label):
            layout.addWidget(w)
        box.setLayout(layout)
        return box

    def _build_site_box(self):
        box = QGroupBox('Site Data')
        layout = QVBoxLayout()
        self.site_id_label = QLabel('Site: --')
        self.gnss_label = QLabel('GNSS: --, --')
        for w in (self.site_id_label, self.gnss_label):
            layout.addWidget(w)
        box.setLayout(layout)
        return box

    def _build_instrument_box(self):
        box = QGroupBox('Instrumentation')
        grid = QGridLayout()
        self.temp_value = QLabel('--')
        self.humidity_value = QLabel('--')
        self.ph_value = QLabel('--')
        self.life_value = QLabel('--')

        rows = [
            ('Temperature (C):', self.temp_value),
            ('Humidity (%):', self.humidity_value),
            ('pH:', self.ph_value),
            ('Life Detection Signal (0-1):', self.life_value),
        ]
        for row, (name, value_widget) in enumerate(rows):
            name_label = QLabel(name)
            name_label.setFont(QFont('Arial', 10))
            value_widget.setFont(QFont('Arial', 10, QFont.Bold))
            grid.addWidget(name_label, row, 0)
            grid.addWidget(value_widget, row, 1)

        box.setLayout(grid)
        return box

    def _build_checklist_box(self):
        box = QGroupBox('Site Documentation Checklist (current site) — URC rule 1.b.iii')
        layout = QVBoxLayout()

        self.checklist_site_label = QLabel('No active site yet.')
        self.checklist_site_label.setFont(QFont('Arial', 10, QFont.Bold))
        layout.addWidget(self.checklist_site_label)

        self.checklist_boxes = {}
        for key, label_text in CHECKLIST_ITEMS:
            cb = QCheckBox(label_text)
            cb.stateChanged.connect(lambda state, k=key: self.on_checklist_changed(k, state))
            layout.addWidget(cb)
            self.checklist_boxes[key] = cb

        self.sample_ready_label = QLabel('Sample-ready: NO (0/4 documented)')
        self.sample_ready_label.setFont(QFont('Arial', 10, QFont.Bold))
        layout.addWidget(self.sample_ready_label)

        box.setLayout(layout)
        return box

    def _build_comparison_table_box(self):
        box = QGroupBox('Multi-Site Comparison')
        layout = QVBoxLayout()
        self.site_table = QTableWidget(0, 6)
        self.site_table.setHorizontalHeaderLabels(
            ['Site', 'Temp (C)', 'Humidity (%)', 'pH', 'Life Signal', 'Checklist']
        )
        self.site_table.horizontalHeader().setStretchLastSection(True)
        self.site_table.setFixedHeight(140)
        layout.addWidget(self.site_table)
        box.setLayout(layout)
        return box

    def _build_sample_box(self):
        box = QGroupBox('Sample Operations')
        layout = QVBoxLayout()

        self.sample_state_label = QLabel('State: IDLE')
        self.sample_state_label.setFont(QFont('Arial', 12, QFont.Bold))
        layout.addWidget(self.sample_state_label)

        self.sample_mass_label = QLabel('Last sample mass: --')
        layout.addWidget(self.sample_mass_label)

        btn_row = QHBoxLayout()
        self.begin_button = QPushButton('Begin Sample Collection')
        self.begin_button.clicked.connect(self.on_begin_clicked)
        self.abort_button = QPushButton('Abort')
        self.abort_button.clicked.connect(self.on_abort_clicked)
        btn_row.addWidget(self.begin_button)
        btn_row.addWidget(self.abort_button)
        layout.addLayout(btn_row)

        self.cache_countdown_label = QLabel('')
        self.cache_countdown_label.setFont(QFont('Arial', 11, QFont.Bold))
        layout.addWidget(self.cache_countdown_label)

        self.cache_removed_button = QPushButton('Cache Removed / Handed to Judges')
        self.cache_removed_button.clicked.connect(self.on_cache_removed_clicked)
        self.cache_removed_button.setVisible(False)
        layout.addWidget(self.cache_removed_button)

        box.setLayout(layout)
        return box

    def _build_log_box(self):
        box = QGroupBox('Command Log')
        layout = QVBoxLayout()
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFixedHeight(120)
        layout.addWidget(self.log_view)
        box.setLayout(layout)
        return box

    # --- behavior -------------------------------------------------------

    def log(self, text):
        timestamp = time.strftime('%H:%M:%S')
        self.log_view.append(f'[{timestamp}] {text}')

    def on_begin_clicked(self):
        self.ros_node.send_command('BEGIN_SAMPLE_COLLECTION')
        self.log('Operator sent BEGIN_SAMPLE_COLLECTION')

    def on_abort_clicked(self):
        self.ros_node.send_command('ABORT')
        self.log('Operator sent ABORT')

    def on_checklist_changed(self, key, state):
        if self.current_site_id is None:
            return
        record = self.site_records.setdefault(
            self.current_site_id, {'data': {}, 'checklist': {k: False for k, _ in CHECKLIST_ITEMS}}
        )
        record['checklist'][key] = bool(state)
        self.log(f"Checklist updated for {self.current_site_id}: {key} = {bool(state)}")

    def on_cache_removed_clicked(self):
        self.cache_removal_deadline = None
        self.cache_countdown_label.setText('')
        self.cache_removed_button.setVisible(False)
        self.log('Cache removed and handed to judges.')

    def refresh_display(self):
        self._refresh_clock()
        self._refresh_comms_and_science()
        self._refresh_checklist_panel()
        self._refresh_comparison_table()
        self._refresh_sample_state()
        self._refresh_cache_countdown()

    def _refresh_clock(self):
        elapsed = int(time.time() - self.start_time)
        minutes, seconds = divmod(elapsed, 60)
        self.clock_label.setText(f'Mission Elapsed: {minutes:02d}:{seconds:02d}')

    def _refresh_comms_and_science(self):
        data = self.ros_node.latest_science_data
        last_time = self.ros_node.last_science_msg_time

        if data is None or last_time is None:
            self.link_status_label.setText('Status: waiting for data...')
            self.link_status_label.setStyleSheet('color: orange;')
            return

        age = time.time() - last_time
        rate = self.ros_node.estimate_rate_hz()
        self.link_rate_label.setText(f'Rate: {rate:.1f} Hz')
        self.link_age_label.setText(f'Last update: {age:.1f}s ago')

        if age > SIGNAL_TIMEOUT_SEC:
            self.link_status_label.setText('Status: SIGNAL LOST')
            self.link_status_label.setStyleSheet('color: red; font-weight: bold;')
            return

        self.link_status_label.setText('Status: LIVE')
        self.link_status_label.setStyleSheet('color: green;')

        site_id = data.get('site_id', '--')
        self.current_site_id = site_id
        self.site_id_label.setText(f'Site: {site_id}')
        self.gnss_label.setText(
            f"GNSS: {data.get('gnss_lat', '--')}, {data.get('gnss_lon', '--')}"
        )
        self.temp_value.setText(f"{data.get('temperature_c', '--')}")
        self.humidity_value.setText(f"{data.get('humidity_pct', '--')}")
        self.ph_value.setText(f"{data.get('ph', '--')}")

        life = data.get('life_detection_signal')
        self.life_value.setText(f"{life}")
        if life is not None and life >= LIFE_SIGNAL_ALERT_THRESHOLD:
            self.life_value.setStyleSheet('color: red; font-weight: bold;')
        else:
            self.life_value.setStyleSheet('')

        # Track this site's latest reading, preserving any existing checklist
        record = self.site_records.setdefault(
            site_id, {'data': {}, 'checklist': {k: False for k, _ in CHECKLIST_ITEMS}}
        )
        record['data'] = data

    def _refresh_checklist_panel(self):
        if self.current_site_id is None:
            return

        record = self.site_records.get(self.current_site_id)
        if record is None:
            return

        self.checklist_site_label.setText(f'Documenting: {self.current_site_id}')

        # Sync checkboxes to the stored state without re-triggering writes
        for key, cb in self.checklist_boxes.items():
            checked = record['checklist'].get(key, False)
            if cb.isChecked() != checked:
                cb.blockSignals(True)
                cb.setChecked(checked)
                cb.blockSignals(False)

        done_count = sum(1 for v in record['checklist'].values() if v)
        total = len(CHECKLIST_ITEMS)
        ready = done_count == total
        self.sample_ready_label.setText(
            f"Sample-ready: {'YES' if ready else 'NO'} ({done_count}/{total} documented)"
        )
        self.sample_ready_label.setStyleSheet('color: green;' if ready else 'color: orange;')

    def _refresh_comparison_table(self):
        site_ids = sorted(self.site_records.keys())
        self.site_table.setRowCount(len(site_ids))

        for row, site_id in enumerate(site_ids):
            record = self.site_records[site_id]
            data = record['data']
            checklist = record['checklist']
            done_count = sum(1 for v in checklist.values() if v)
            ready = done_count == len(CHECKLIST_ITEMS)

            values = [
                site_id,
                str(data.get('temperature_c', '--')),
                str(data.get('humidity_pct', '--')),
                str(data.get('ph', '--')),
                str(data.get('life_detection_signal', '--')),
                f'{done_count}/{len(CHECKLIST_ITEMS)}' + (' — READY' if ready else ''),
            ]

            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                if ready:
                    item.setBackground(QColor(200, 255, 200))
                self.site_table.setItem(row, col, item)

        self.site_table.resizeColumnsToContents()

    def _refresh_sample_state(self):
        status = self.ros_node.latest_status
        state = status.get('state', 'IDLE')
        mass = status.get('sample_mass_g')

        self.sample_state_label.setText(f'State: {state}')
        color_map = {
            'IDLE': 'black',
            'DRILLING': 'orange',
            'ANALYZING': 'blue',
            'COMPLETE': 'green',
        }
        self.sample_state_label.setStyleSheet(f"color: {color_map.get(state, 'black')};")

        self.sample_mass_label.setText(
            f'Last sample mass: {mass} g' if mass is not None else 'Last sample mass: --'
        )

        self.begin_button.setEnabled(state == 'IDLE')

        if state != self.last_logged_state:
            self.log(f'Rover reported state change: {state}')
            if state == 'COMPLETE' and self.cache_removal_deadline is None:
                self.cache_removal_deadline = time.time() + CACHE_REMOVAL_WINDOW_SEC
                self.cache_removed_button.setVisible(True)
                self.log('Cache ready. 5:00 removal window started (rule 1.b.vii).')
            self.last_logged_state = state

    def _refresh_cache_countdown(self):
        if self.cache_removal_deadline is None:
            return

        remaining = self.cache_removal_deadline - time.time()
        if remaining <= 0:
            self.cache_countdown_label.setText('Cache removal window: OVERDUE')
            self.cache_countdown_label.setStyleSheet('color: red;')
            return

        minutes, seconds = divmod(int(remaining), 60)
        self.cache_countdown_label.setText(
            f'Cache removal window: {minutes:01d}:{seconds:02d} remaining'
        )
        self.cache_countdown_label.setStyleSheet(
            'color: red;' if remaining < 60 else 'color: black;'
        )


def main():
    rclpy.init()
    ros_node = ScienceGuiNode()

    app = QApplication(sys.argv)
    window = ScienceConsole(ros_node)
    window.show()

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
