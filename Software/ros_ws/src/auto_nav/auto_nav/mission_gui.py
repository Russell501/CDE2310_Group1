#!/usr/bin/env python3
"""
mission_gui.py — Live mission status dashboard for CDE2310 Group 1.

Run with:
    ros2 run auto_nav mission_gui
"""

import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Int32
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from explore_lite_msgs.msg import ExploreStatus

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
# COLOURS
# =============================================================================
BG       = '#1a1a2e'
PANEL    = '#16213e'
ACCENT   = '#0f3460'
TEXT     = '#e8e8e8'
TEXT_DIM = '#666666'

C_IDLE    = '#444444'   # grey   — not yet started
C_RUNNING = '#f5a623'   # amber  — in progress
C_DONE    = '#27ae60'   # green  — complete
C_WARN    = '#e74c3c'   # red    — re-mapping / warning


# =============================================================================
# SHARED STATE  (written by ROS thread, read by GUI thread)
# =============================================================================
class State:
    def __init__(self):
        self.cartographer  = 'idle'     # idle | running
        self.nav2          = 'idle'     # idle | running
        self.explore_lite  = 'idle'     # idle | running | done
        self.mapping       = 'idle'     # idle | running | done
        self.aruco_ids     = set()      # dock marker IDs only (0, 1)
        self.frontiers_done = False
        self.remap_count   = 0
        self.remap_running = False
        self.bfs           = 'idle'     # idle | running | done
        self.phase         = ''         # raw phase string from FSM
        # Station B
        self.trigger_detections = 0     # how many times trigger marker seen
        self.station_b_balls    = 0     # balls fired at Station B


# =============================================================================
# ROS 2 MONITOR NODE
# =============================================================================
class MissionMonitorNode(Node):
    def __init__(self, state: State):
        super().__init__('mission_gui_monitor')
        self.state = state

        # Cartographer — detect via /map
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)

        # Nav2 — detect via local costmap
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._nav2_cb, 10)

        # explore_lite status
        self.create_subscription(
            ExploreStatus, 'explore/status', self._explore_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # ArUco detections
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            MarkerArray, '/aruco/markers', self._aruco_cb, qos_be)

        # FSM phase signals
        self.create_subscription(String, '/mission_phase', self._phase_cb, 10)

        # Station B: trigger detections and balls fired (published by FSM)
        self.create_subscription(
            Int32, '/station_b_trigger_count', self._trigger_count_cb, 10)
        self.create_subscription(
            Int32, '/station_b_balls_fired', self._balls_fired_cb, 10)

    # -------------------------------------------------------------------------
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
                elif marker.id == STATION_B_TRIGGER_MARKER_ID:
                    # Trigger detections are counted via /station_b_trigger_count
                    # but we can also bump here as a fallback
                    pass

    def _phase_cb(self, msg):
        s = self.state
        phase = msg.data
        s.phase = phase

        if phase == 'EXPLORE':
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


# =============================================================================
# GUI
# =============================================================================
class MissionGUI:
    def __init__(self, root: tk.Tk, state: State):
        self.root  = root
        self.state = state

        root.title('CDE2310 Group 1 — Mission Control')
        root.configure(bg=BG)
        root.resizable(False, False)

        # ── Title bar ────────────────────────────────────────────────────────
        tk.Label(
            root, text='  MISSION STATUS', bg=ACCENT, fg=TEXT,
            font=('Helvetica', 14, 'bold'), anchor='w', pady=10, padx=10,
        ).pack(fill='x')

        # ── Status rows ──────────────────────────────────────────────────────
        self._rows = {}
        container = tk.Frame(root, bg=BG, padx=28, pady=12)
        container.pack(fill='both')

        items = [
            ('cartographer', '1.  Cartographer'),
            ('nav2',         '2.  Nav2'),
            ('explore_lite', '3.  explore_lite'),
            ('mapping',      '4.  Mapping'),
            ('aruco',        '5.  ArUco Dock Markers'),
            ('frontiers',    '6.  Frontiers Exhausted'),
            ('remap',        '7.  Re-mapping'),
            ('bfs',          '8.  BFS Traversal'),
            ('stb_trigger',  '9.  Station B Trigger'),
            ('stb_balls',    '10. Station B Balls'),
        ]

        for key, label in items:
            row = tk.Frame(container, bg=BG, pady=5)
            row.pack(fill='x')

            canvas = tk.Canvas(row, width=16, height=16, bg=BG,
                               highlightthickness=0)
            canvas.pack(side='left', padx=(0, 14))
            dot = canvas.create_oval(1, 1, 15, 15, fill=C_IDLE, outline='')

            tk.Label(row, text=label, bg=BG, fg=TEXT,
                     font=('Helvetica', 11), width=24, anchor='w').pack(side='left')

            badge = tk.Label(row, text='—', bg=PANEL, fg=TEXT_DIM,
                             font=('Courier', 10), padx=10, pady=3,
                             width=28, anchor='w')
            badge.pack(side='left')

            self._rows[key] = (canvas, dot, badge)

        # ── Divider + phase footer ────────────────────────────────────────────
        tk.Frame(root, bg=ACCENT, height=1).pack(fill='x', padx=20, pady=(2, 0))
        self._footer = tk.Label(
            root, text='Waiting for ROS 2...', bg=BG, fg=TEXT_DIM,
            font=('Helvetica', 9, 'italic'), pady=8)
        self._footer.pack()

        self._refresh()

    # ── helpers ──────────────────────────────────────────────────────────────
    def _set(self, key, color, text):
        canvas, dot, badge = self._rows[key]
        canvas.itemconfig(dot, fill=color)
        badge.config(
            text=text,
            fg=TEXT if color != C_IDLE else TEXT_DIM,
        )

    def _refresh(self):
        s = self.state

        # 1. Cartographer
        if s.cartographer == 'running':
            self._set('cartographer', C_RUNNING, 'RUNNING')
        else:
            self._set('cartographer', C_IDLE, '—')

        # 2. Nav2
        if s.nav2 == 'running':
            self._set('nav2', C_RUNNING, 'RUNNING')
        else:
            self._set('nav2', C_IDLE, '—')

        # 3. explore_lite
        status_map = {
            'running': (C_RUNNING, 'RUNNING'),
            'done':    (C_DONE,    'STOPPED'),
            'idle':    (C_IDLE,    '—'),
        }
        self._set('explore_lite', *status_map[s.explore_lite])

        # 4. Mapping
        mapping_map = {
            'running': (C_RUNNING, 'IN PROGRESS'),
            'done':    (C_DONE,    'COMPLETE'),
            'idle':    (C_IDLE,    '—'),
        }
        self._set('mapping', *mapping_map[s.mapping])

        # 5. ArUco Dock Markers (IDs 0 and 1 only)
        count   = len(s.aruco_ids)
        ids_str = '  '.join(f'ID:{i}' for i in sorted(s.aruco_ids))
        if count >= REQUIRED_MARKER_COUNT:
            self._set('aruco', C_DONE,
                      f'{count}/{REQUIRED_MARKER_COUNT} found   [{ids_str}]')
        elif count > 0:
            self._set('aruco', C_RUNNING,
                      f'{count}/{REQUIRED_MARKER_COUNT} found   [{ids_str}]')
        else:
            self._set('aruco', C_IDLE, f'0 / {REQUIRED_MARKER_COUNT}')

        # 6. Frontiers Exhausted
        if s.frontiers_done:
            self._set('frontiers', C_DONE, 'YES — no frontiers remain')
        else:
            self._set('frontiers', C_IDLE, '—')

        # 7. Re-mapping
        if s.remap_count == 0:
            self._set('remap', C_IDLE, '—')
        elif s.remap_running:
            self._set('remap', C_WARN,
                      f'ATTEMPT {s.remap_count} / {MAX_EXPLORE_RESTARTS}  RUNNING')
        else:
            self._set('remap', C_DONE,
                      f'{s.remap_count} restart(s) — complete')

        # 8. BFS Traversal
        bfs_map = {
            'running': (C_RUNNING, 'IN PROGRESS'),
            'done':    (C_DONE,    'COMPLETE'),
            'idle':    (C_IDLE,    '—'),
        }
        self._set('bfs', *bfs_map[s.bfs])

        # 9. Station B Trigger (ID=2 moving receptacle marker)
        tc = s.trigger_detections
        if s.phase in ('DOCK_B', 'COMPLETE') or tc > 0:
            if tc > 0:
                self._set('stb_trigger', C_RUNNING,
                          f'ID:{STATION_B_TRIGGER_MARKER_ID}  seen x{tc}')
            else:
                self._set('stb_trigger', C_RUNNING, 'Waiting for trigger...')
        else:
            self._set('stb_trigger', C_IDLE, '—')

        # 10. Station B Balls Fired
        fired = s.station_b_balls
        if fired >= STATION_B_BALLS:
            self._set('stb_balls', C_DONE,
                      f'{fired} / {STATION_B_BALLS}  ALL FIRED')
        elif fired > 0:
            self._set('stb_balls', C_RUNNING,
                      f'{fired} / {STATION_B_BALLS}  fired')
        elif s.phase == 'DOCK_B':
            self._set('stb_balls', C_RUNNING, f'0 / {STATION_B_BALLS}  pending')
        else:
            self._set('stb_balls', C_IDLE, f'0 / {STATION_B_BALLS}')

        # Footer
        phase_labels = {
            'EXPLORE':  'Phase 1 — Exploration underway',
            'REMAP':    'Phase 1 — Re-mapping (explore_lite restarted)',
            'SWEEP':    'Phase 2 — BFS coverage sweep',
            'DOCK_A':   'Phase 3 — Docking at Station A (stationary)',
            'DOCK_B':   'Phase 4 — Docking at Station B (moving receptacle)',
            'COMPLETE': 'All missions complete!',
        }
        if s.phase in phase_labels:
            self._footer.config(
                text=phase_labels[s.phase],
                fg=C_DONE if s.phase == 'COMPLETE' else TEXT_DIM,
            )

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
