#!/usr/bin/env python3
"""
mission_gui.py — Live mission status dashboard for CDE2310 Group 1.

Run with:
    ros2 run auto_nav mission_gui
"""

import collections
import threading
import time as _time
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Int32
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from explore_lite_msgs.msg import ExploreStatus
from rcl_interfaces.msg import Log

# Import constants from FSM so GUI stays in sync
try:
    from auto_nav.exploration_fsm import (
        MAX_EXPLORE_RESTARTS,
        STATION_A_MARKER_ID,
        STATION_B_MARKER_ID,
        STATION_B_TRIGGER_MARKER_ID,
        STATION_B_BALLS,
    )
except ImportError:
    MAX_EXPLORE_RESTARTS        = 2
    STATION_A_MARKER_ID         = 0
    STATION_B_MARKER_ID         = 1
    STATION_B_TRIGGER_MARKER_ID = 2
    STATION_B_BALLS             = 3

REQUIRED_MARKER_COUNT = 2   # dock markers only (IDs 0 and 1)
DOCK_MARKER_IDS = {STATION_A_MARKER_ID, STATION_B_MARKER_ID}


# =============================================================================
# COLOUR PALETTE
# =============================================================================
BG         = '#0f111a'
BG_CARD    = '#1a1e2e'
BG_HEADER  = '#1e2340'
BG_LOG     = '#0a0c14'
BORDER     = '#2a2f45'
ACCENT     = '#3d5af1'
ACCENT_DIM = '#2a3eb1'
TEXT       = '#e4e8f1'
TEXT_SEC   = '#8892b0'
TEXT_DIM   = '#4a5068'

C_IDLE    = '#3a3f55'
C_RUNNING = '#f0a500'
C_DONE    = '#00d68f'
C_WARN    = '#ff6b6b'


# =============================================================================
# SHARED STATE  (written by ROS thread, read by GUI thread)
# =============================================================================
class State:
    def __init__(self):
        self.cartographer  = 'idle'
        self.nav2          = 'idle'
        self.explore_lite  = 'idle'
        self.mapping       = 'idle'
        self.aruco_ids     = set()
        self.frontiers_done = False
        self.remap_count   = 0
        self.remap_running = False
        self.bfs           = 'idle'
        self.phase         = ''
        self.trigger_detections = 0
        self.station_b_balls    = 0
        self.logs      = collections.deque(maxlen=200)
        self.logs_lock = threading.Lock()
        self.start_time = _time.monotonic()


# =============================================================================
# ROS 2 MONITOR NODE
# =============================================================================
class MissionMonitorNode(Node):
    def __init__(self, state: State):
        super().__init__('mission_gui_monitor')
        self.state = state

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._nav2_cb, map_qos)
        self.create_subscription(
            ExploreStatus, 'explore/status', self._explore_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            MarkerArray, '/aruco/markers', self._aruco_cb, qos_be)
        self.create_subscription(String, '/mission_phase', self._phase_cb, 10)
        self.create_subscription(
            Int32, '/station_b_trigger_count', self._trigger_count_cb, 10)
        self.create_subscription(
            Int32, '/station_b_balls_fired', self._balls_fired_cb, 10)
        self.create_subscription(
            Log, '/rosout', self._rosout_cb,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _map_cb(self, msg):
        if self.state.cartographer == 'idle':
            self.state.cartographer = 'running'

    def _nav2_cb(self, msg):
        if self.state.nav2 == 'idle':
            self.state.nav2 = 'running'

    def _explore_cb(self, msg):
        s = self.state
        if msg.status == ExploreStatus.EXPLORATION_COMPLETE:
            s.explore_lite    = 'done'
            s.frontiers_done  = True
            s.remap_running   = False
            if s.mapping == 'running':
                s.mapping = 'done'
        else:
            if s.explore_lite != 'running':
                s.explore_lite = 'running'
            if s.mapping == 'idle':
                s.mapping = 'running'

    def _aruco_cb(self, msg):
        for marker in msg.markers:
            if marker.pose.position.z > 0.0:
                if marker.id in DOCK_MARKER_IDS:
                    self.state.aruco_ids.add(marker.id)

    def _phase_cb(self, msg):
        s = self.state
        phase = msg.data
        s.phase = phase

        if phase == 'INITIAL_BFS':
            s.bfs = 'running'
            if s.mapping == 'idle':
                s.mapping = 'running'
        elif phase == 'EXPLORE':
            if s.bfs == 'running':
                s.bfs = 'done'
            if s.explore_lite == 'idle':
                s.explore_lite = 'running'
            if s.mapping == 'idle':
                s.mapping = 'running'
        elif phase == 'REMAP':
            s.remap_count  += 1
            s.remap_running = True
            s.explore_lite  = 'running'
            s.frontiers_done = True
        elif phase == 'SWEEP':
            s.bfs            = 'running'
            s.explore_lite   = 'done'
            s.mapping        = 'done'
            s.frontiers_done = True
            s.remap_running  = False
        elif phase in ('DOCK_A', 'DOCK_B'):
            if s.bfs == 'running':
                s.bfs = 'done'
        elif phase == 'COMPLETE':
            if s.bfs == 'running':
                s.bfs = 'done'

    def _trigger_count_cb(self, msg):
        self.state.trigger_detections = msg.data

    def _balls_fired_cb(self, msg):
        self.state.station_b_balls = msg.data

    def _rosout_cb(self, msg: Log):
        if msg.name != 'ultimate_mission_controller':
            return
        level_map = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
        level = level_map.get(msg.level, 'INFO')
        with self.state.logs_lock:
            self.state.logs.append((level, msg.msg))


# =============================================================================
# HELPER — rounded rectangle card
# =============================================================================
def _make_card(parent, **kw):
    """Return a Frame styled as a card."""
    f = tk.Frame(parent, bg=BG_CARD, highlightbackground=BORDER,
                 highlightthickness=1, **kw)
    return f


# =============================================================================
# GUI
# =============================================================================
class MissionGUI:
    def __init__(self, root: tk.Tk, state: State):
        self.root  = root
        self.state = state
        self._pulse_on = True

        root.title('CDE2310 Group 1 — Mission Control')
        root.configure(bg=BG)
        root.resizable(False, False)
        root.option_add('*Font', 'TkDefaultFont')

        # ── Header ───────────────────────────────────────────────────────────
        hdr = tk.Frame(root, bg=BG_HEADER, pady=14, padx=20)
        hdr.pack(fill='x')

        tk.Label(
            hdr, text='MISSION CONTROL', bg=BG_HEADER, fg=TEXT,
            font=('Helvetica', 16, 'bold'),
        ).pack(side='left')

        tk.Label(
            hdr, text='CDE2310 Group 1', bg=BG_HEADER, fg=TEXT_SEC,
            font=('Helvetica', 10),
        ).pack(side='left', padx=(12, 0), pady=(4, 0))

        self._timer_label = tk.Label(
            hdr, text='00:00', bg=BG_HEADER, fg=ACCENT,
            font=('Courier', 14, 'bold'),
        )
        self._timer_label.pack(side='right')

        tk.Label(
            hdr, text='ELAPSED', bg=BG_HEADER, fg=TEXT_DIM,
            font=('Helvetica', 8),
        ).pack(side='right', padx=(0, 8), pady=(4, 0))

        # ── Phase banner ─────────────────────────────────────────────────────
        self._phase_banner = tk.Label(
            root, text='  Waiting for ROS 2...', bg=ACCENT_DIM, fg=TEXT,
            font=('Helvetica', 11, 'bold'), anchor='w', pady=8, padx=16,
        )
        self._phase_banner.pack(fill='x')

        # ── Main content ─────────────────────────────────────────────────────
        main = tk.Frame(root, bg=BG, padx=12, pady=12)
        main.pack(fill='both', expand=True)

        # ── Left: Status cards ───────────────────────────────────────────────
        left = tk.Frame(main, bg=BG)
        left.pack(side='left', fill='y')

        # -- Systems card --
        sys_card = _make_card(left, padx=16, pady=12)
        sys_card.pack(fill='x', pady=(0, 8))
        tk.Label(sys_card, text='SYSTEMS', bg=BG_CARD, fg=TEXT_SEC,
                 font=('Helvetica', 9, 'bold')).pack(anchor='w', pady=(0, 8))

        self._rows = {}
        sys_items = [
            ('cartographer', 'Cartographer'),
            ('nav2',         'Nav2'),
            ('explore_lite', 'explore_lite'),
            ('mapping',      'Mapping'),
        ]
        for key, label in sys_items:
            self._make_status_row(sys_card, key, label)

        # -- Detection card --
        det_card = _make_card(left, padx=16, pady=12)
        det_card.pack(fill='x', pady=(0, 8))
        tk.Label(det_card, text='DETECTION', bg=BG_CARD, fg=TEXT_SEC,
                 font=('Helvetica', 9, 'bold')).pack(anchor='w', pady=(0, 8))

        det_items = [
            ('aruco',     'ArUco Markers'),
            ('frontiers', 'Frontiers'),
            ('remap',     'Re-mapping'),
            ('bfs',       'BFS Sweep'),
        ]
        for key, label in det_items:
            self._make_status_row(det_card, key, label)

        # -- Station B card --
        stb_card = _make_card(left, padx=16, pady=12)
        stb_card.pack(fill='x')
        tk.Label(stb_card, text='STATION B', bg=BG_CARD, fg=TEXT_SEC,
                 font=('Helvetica', 9, 'bold')).pack(anchor='w', pady=(0, 8))

        stb_items = [
            ('stb_trigger', 'Trigger'),
            ('stb_balls',   'Balls Fired'),
        ]
        for key, label in stb_items:
            self._make_status_row(stb_card, key, label)

        # ── Right: Log panel ─────────────────────────────────────────────────
        log_card = _make_card(main, padx=12, pady=12)
        log_card.pack(side='left', fill='both', expand=True, padx=(10, 0))

        log_hdr = tk.Frame(log_card, bg=BG_CARD)
        log_hdr.pack(fill='x', pady=(0, 8))
        tk.Label(log_hdr, text='FSM LOG', bg=BG_CARD, fg=TEXT_SEC,
                 font=('Helvetica', 9, 'bold')).pack(side='left')

        self._log_count_label = tk.Label(
            log_hdr, text='0 entries', bg=BG_CARD, fg=TEXT_DIM,
            font=('Helvetica', 8))
        self._log_count_label.pack(side='right')

        log_inner = tk.Frame(log_card, bg=BG_LOG, highlightbackground=BORDER,
                             highlightthickness=1)
        log_inner.pack(fill='both', expand=True)

        scrollbar = tk.Scrollbar(log_inner, troughcolor=BG_LOG,
                                 bg=BORDER, activebackground=TEXT_DIM,
                                 width=10, relief='flat')
        scrollbar.pack(side='right', fill='y')

        self._log_text = tk.Text(
            log_inner, bg=BG_LOG, fg=TEXT,
            font=('Courier', 9), width=55, height=26,
            state='disabled', wrap='word', borderwidth=0,
            padx=8, pady=6, spacing1=2,
            yscrollcommand=scrollbar.set,
            insertbackground=TEXT,
            selectbackground=ACCENT_DIM,
        )
        self._log_text.pack(side='left', fill='both', expand=True)
        scrollbar.config(command=self._log_text.yview)

        self._log_text.tag_config('DEBUG', foreground=TEXT_DIM)
        self._log_text.tag_config('INFO',  foreground='#a8b2d1')
        self._log_text.tag_config('WARN',  foreground=C_RUNNING)
        self._log_text.tag_config('ERROR', foreground=C_WARN)
        self._log_text.tag_config('FATAL', foreground=C_WARN)
        self._log_text.tag_config('ts',    foreground=TEXT_DIM)

        self._log_rendered = 0

        self._refresh()

    # ── Row builder ──────────────────────────────────────────────────────────
    def _make_status_row(self, parent, key, label):
        row = tk.Frame(parent, bg=BG_CARD, pady=4)
        row.pack(fill='x')

        # Status indicator (pill shape via a wider canvas)
        canvas = tk.Canvas(row, width=10, height=10, bg=BG_CARD,
                           highlightthickness=0)
        canvas.pack(side='left', padx=(0, 10), pady=(3, 0))
        dot = canvas.create_oval(0, 0, 10, 10, fill=C_IDLE, outline='')

        tk.Label(row, text=label, bg=BG_CARD, fg=TEXT,
                 font=('Helvetica', 10), width=16, anchor='w').pack(side='left')

        badge = tk.Label(row, text='--', bg=BG, fg=TEXT_DIM,
                         font=('Courier', 9), padx=8, pady=2,
                         width=26, anchor='w',
                         highlightbackground=BORDER, highlightthickness=1)
        badge.pack(side='right')

        self._rows[key] = (canvas, dot, badge)

    # ── helpers ──────────────────────────────────────────────────────────────
    def _set(self, key, color, text):
        canvas, dot, badge = self._rows[key]
        canvas.itemconfig(dot, fill=color)
        badge.config(text=text, fg=TEXT if color != C_IDLE else TEXT_DIM)

    def _refresh(self):
        s = self.state
        self._pulse_on = not self._pulse_on

        # Timer
        elapsed = int(_time.monotonic() - s.start_time)
        mins, secs = divmod(elapsed, 60)
        self._timer_label.config(text=f'{mins:02d}:{secs:02d}')

        # 1. Cartographer
        if s.cartographer == 'running':
            self._set('cartographer', C_DONE, 'ACTIVE')
        else:
            self._set('cartographer', C_IDLE, '--')

        # 2. Nav2
        if s.nav2 == 'running':
            self._set('nav2', C_DONE, 'ACTIVE')
        else:
            self._set('nav2', C_IDLE, '--')

        # 3. explore_lite
        status_map = {
            'running': (C_RUNNING, 'RUNNING'),
            'done':    (C_DONE,    'COMPLETE'),
            'idle':    (C_IDLE,    '--'),
        }
        self._set('explore_lite', *status_map[s.explore_lite])

        # 4. Mapping
        mapping_map = {
            'running': (C_RUNNING, 'IN PROGRESS'),
            'done':    (C_DONE,    'COMPLETE'),
            'idle':    (C_IDLE,    '--'),
        }
        self._set('mapping', *mapping_map[s.mapping])

        # 5. ArUco Dock Markers
        count   = len(s.aruco_ids)
        ids_str = ', '.join(f'ID {i}' for i in sorted(s.aruco_ids))
        if count >= REQUIRED_MARKER_COUNT:
            self._set('aruco', C_DONE,
                      f'{count}/{REQUIRED_MARKER_COUNT}  [{ids_str}]')
        elif count > 0:
            self._set('aruco', C_RUNNING,
                      f'{count}/{REQUIRED_MARKER_COUNT}  [{ids_str}]')
        else:
            self._set('aruco', C_IDLE, f'0/{REQUIRED_MARKER_COUNT}')

        # 6. Frontiers Exhausted
        if s.frontiers_done:
            self._set('frontiers', C_DONE, 'EXHAUSTED')
        else:
            self._set('frontiers', C_IDLE, '--')

        # 7. Re-mapping
        if s.remap_count == 0:
            self._set('remap', C_IDLE, '--')
        elif s.remap_running:
            c = C_WARN if self._pulse_on else C_RUNNING
            self._set('remap', c,
                      f'RUN {s.remap_count}/{MAX_EXPLORE_RESTARTS}')
        else:
            self._set('remap', C_DONE,
                      f'{s.remap_count} restart(s)')

        # 8. BFS Traversal
        bfs_map = {
            'running': (C_RUNNING, 'IN PROGRESS'),
            'done':    (C_DONE,    'COMPLETE'),
            'idle':    (C_IDLE,    '--'),
        }
        self._set('bfs', *bfs_map[s.bfs])

        # 9. Station B Trigger
        tc = s.trigger_detections
        if s.phase in ('DOCK_B', 'COMPLETE') or tc > 0:
            if tc > 0:
                self._set('stb_trigger', C_DONE,
                          f'ID {STATION_B_TRIGGER_MARKER_ID}  x{tc}')
            else:
                c = C_RUNNING if self._pulse_on else C_IDLE
                self._set('stb_trigger', c, 'Waiting...')
        else:
            self._set('stb_trigger', C_IDLE, '--')

        # 10. Station B Balls Fired
        fired = s.station_b_balls
        if fired >= STATION_B_BALLS:
            self._set('stb_balls', C_DONE,
                      f'{fired}/{STATION_B_BALLS}  DONE')
        elif fired > 0:
            self._set('stb_balls', C_RUNNING,
                      f'{fired}/{STATION_B_BALLS}')
        elif s.phase == 'DOCK_B':
            self._set('stb_balls', C_RUNNING, f'0/{STATION_B_BALLS}  pending')
        else:
            self._set('stb_balls', C_IDLE, f'0/{STATION_B_BALLS}')

        # Phase banner
        phase_labels = {
            'INITIAL_BFS':       'Phase 1a  -  Initial BFS (map seeding)',
            'EXPLORE':           'Phase 1b  -  explore_lite',
            'REMAP':             'Phase 1b  -  Re-mapping',
            'FRONTIER_CLEANUP':  'Phase 1c  -  Frontier cleanup',
            'SWEEP':             'Phase 2   -  BFS coverage sweep',
            'DOCK_A':            'Phase 3   -  Docking at Station A',
            'DOCK_B':            'Phase 4   -  Docking at Station B',
            'COMPLETE':          'Mission complete',
            'ABORTED':           'Mission aborted',
        }
        if s.phase in phase_labels:
            label = phase_labels[s.phase]
            if s.phase == 'COMPLETE':
                bg, fg = C_DONE, '#000000'
            elif s.phase == 'ABORTED':
                bg, fg = C_WARN, TEXT
            else:
                bg, fg = ACCENT_DIM, TEXT
            self._phase_banner.config(text=f'  {label}', bg=bg, fg=fg)

        # Log panel
        with s.logs_lock:
            snapshot = list(s.logs)
        new_entries = snapshot[self._log_rendered:]
        self._log_rendered = len(snapshot)

        if new_entries:
            self._log_text.config(state='normal')
            for level, msg in new_entries:
                self._log_text.insert('end', f'[{level:5s}] ', 'ts')
                self._log_text.insert('end', f'{msg}\n', level)
            self._log_text.see('end')
            self._log_text.config(state='disabled')

        self._log_count_label.config(text=f'{len(snapshot)} entries')

        self.root.after(500, self._refresh)


# =============================================================================
# ENTRY POINT
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    state = State()
    node  = MissionMonitorNode(state)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    MissionGUI(root, state)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
