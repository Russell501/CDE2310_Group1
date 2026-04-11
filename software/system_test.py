#!/usr/bin/env python3
"""
system_test.py — CDE2310 Group 1 system integration test.

A pre-flight check that verifies all software components are wired up correctly
without running the full mission. Designed to mirror exploration_fsm.py's
ROS-side setup so that "if this passes, the FSM has what it needs to run."

Tests performed (in order):
  1. ROS 2 init + node creation
  2. TF tree present (map -> base_link)
  3. /aruco/markers topic publishing (camera + ArUco pipeline alive)
  4. ArUco marker visible (at least one marker detected)
  5. Nav2 BasicNavigator becomes active (no goals sent)
  6. /start_flywheel service available + callable
  7. /fire_ball service available + callable (fires ONE ball)
  8. /stop_flywheel service available + callable

A Tkinter GUI shows live status. Once all tests have run (pass or fail), the
script tears down the node, shuts down rclpy, and exits cleanly.

Run on the laptop (after SSHing into the RPi to start ball_launcher.py and the
camera/aruco/nav2 stack), the same way you'd run exploration_fsm.py:

    python3 system_test.py
"""

import threading
import time
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger

from nav2_simple_commander.robot_navigator import BasicNavigator


# =============================================================================
# CONFIGURATION
# =============================================================================
TF_TIMEOUT_S            = 10.0
MARKER_TOPIC_TIMEOUT_S  = 8.0
MARKER_DETECT_TIMEOUT_S = 15.0
NAV2_ACTIVE_TIMEOUT_S   = 20.0
SERVICE_WAIT_S          = 5.0
FLYWHEEL_SPINUP_S       = 1.5   # let motors come up to speed before firing
FLYWHEEL_SPINDOWN_S     = 0.5   # gap between fire and stop


# =============================================================================
# TEST RESULT DATA
# =============================================================================
PENDING = 'PENDING'
RUNNING = 'RUNNING'
PASSED  = 'PASSED'
FAILED  = 'FAILED'
SKIPPED = 'SKIPPED'

STATUS_COLORS = {
    PENDING: '#888888',
    RUNNING: '#1e88e5',
    PASSED:  '#2e7d32',
    FAILED:  '#c62828',
    SKIPPED: '#f9a825',
}

@dataclass
class TestResult:
    name: str
    status: str = PENDING
    message: str = ''


# =============================================================================
# TEST NODE
# =============================================================================
class SystemTestNode(Node):
    """Holds all ROS interfaces. Runs on the executor thread."""

    def __init__(self):
        super().__init__('system_test_node')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Marker subscriber
        self._marker_lock = threading.Lock()
        self._marker_msg_count = 0
        self._detected_marker_ids = set()
        self.create_subscription(
            MarkerArray, '/aruco/markers', self._marker_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        # Service clients (matching exploration_fsm.py exactly)
        self.flywheel_start_client = self.create_client(Trigger, '/start_flywheel')
        self.fire_ball_client      = self.create_client(Trigger, '/fire_ball')
        self.flywheel_stop_client  = self.create_client(Trigger, '/stop_flywheel')

    def _marker_cb(self, msg: MarkerArray):
        with self._marker_lock:
            self._marker_msg_count += 1
            for m in msg.markers:
                self._detected_marker_ids.add(m.id)

    @property
    def marker_msg_count(self) -> int:
        with self._marker_lock:
            return self._marker_msg_count

    @property
    def detected_marker_ids(self) -> set:
        with self._marker_lock:
            return set(self._detected_marker_ids)

    def call_service_blocking(self, client, label: str, timeout_s: float = SERVICE_WAIT_S):
        """Returns (success: bool, message: str)."""
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False, f'{label}: service not available within {timeout_s}s'
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                return False, f'{label}: call timed out after {timeout_s}s'
            time.sleep(0.05)
        result = future.result()
        if result is None:
            return False, f'{label}: future returned None'
        if not result.success:
            return False, f'{label}: service reported failure ({result.message})'
        return True, f'{label}: {result.message or "OK"}'


# =============================================================================
# TEST RUNNER
# =============================================================================
class TestRunner:
    """Runs the suite of tests sequentially on a background thread."""

    def __init__(self, node: SystemTestNode, gui: 'TestGUI'):
        self.node = node
        self.gui = gui
        self.results: List[TestResult] = [
            TestResult('1. ROS 2 node initialised'),
            TestResult('2. TF tree (map -> base_link)'),
            TestResult('3. /aruco/markers topic publishing'),
            TestResult('4. ArUco marker visible'),
            TestResult('5. Nav2 stack active'),
            TestResult('6. /start_flywheel service'),
            TestResult('7. /fire_ball service (1 ball)'),
            TestResult('8. /stop_flywheel service'),
        ]
        self._navigator: Optional[BasicNavigator] = None

    # -- helpers ----------------------------------------------------------------
    def _set(self, idx: int, status: str, message: str = ''):
        self.results[idx].status = status
        self.results[idx].message = message
        self.gui.refresh(self.results)

    # -- individual tests -------------------------------------------------------
    def _test_node_init(self, idx: int) -> bool:
        self._set(idx, RUNNING)
        # If we got here, rclpy.init() and node creation already succeeded.
        self._set(idx, PASSED, f"node='{self.node.get_name()}'")
        return True

    def _test_tf_tree(self, idx: int) -> bool:
        self._set(idx, RUNNING, f'waiting up to {TF_TIMEOUT_S}s...')
        deadline = time.monotonic() + TF_TIMEOUT_S
        last_err = ''
        while time.monotonic() < deadline:
            try:
                self.node.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                self._set(idx, PASSED, 'map -> base_link OK')
                return True
            except Exception as e:
                last_err = str(e).splitlines()[0] if str(e) else 'unknown'
                time.sleep(0.5)
        self._set(idx, FAILED, f'timed out: {last_err}')
        return False

    def _test_marker_topic(self, idx: int) -> bool:
        self._set(idx, RUNNING, f'waiting up to {MARKER_TOPIC_TIMEOUT_S}s...')
        deadline = time.monotonic() + MARKER_TOPIC_TIMEOUT_S
        start_count = self.node.marker_msg_count
        while time.monotonic() < deadline:
            if self.node.marker_msg_count > start_count:
                self._set(idx, PASSED, f'received {self.node.marker_msg_count} msg(s)')
                return True
            time.sleep(0.2)
        self._set(idx, FAILED, 'no messages on /aruco/markers — camera or aruco node down?')
        return False

    def _test_marker_visible(self, idx: int) -> bool:
        self._set(idx, RUNNING, f'looking for any marker (up to {MARKER_DETECT_TIMEOUT_S}s)...')
        deadline = time.monotonic() + MARKER_DETECT_TIMEOUT_S
        while time.monotonic() < deadline:
            ids = self.node.detected_marker_ids
            if ids:
                self._set(idx, PASSED, f'detected IDs: {sorted(ids)}')
                return True
            time.sleep(0.3)
        self._set(idx, FAILED, 'no markers detected — point camera at a marker before starting')
        return False

    def _test_nav2_active(self, idx: int) -> bool:
        self._set(idx, RUNNING, f'waiting for Nav2 (up to {NAV2_ACTIVE_TIMEOUT_S}s)...')
        try:
            self._navigator = BasicNavigator()
        except Exception as e:
            self._set(idx, FAILED, f'BasicNavigator construction failed: {e}')
            return False

        # waitUntilNav2Active blocks; run it with a watchdog so we can fail fast.
        done_event = threading.Event()
        err_holder: List[Optional[BaseException]] = [None]

        def _wait():
            try:
                self._navigator.waitUntilNav2Active()
            except BaseException as e:  # noqa: BLE001
                err_holder[0] = e
            finally:
                done_event.set()

        t = threading.Thread(target=_wait, daemon=True)
        t.start()
        if not done_event.wait(timeout=NAV2_ACTIVE_TIMEOUT_S):
            self._set(idx, FAILED, 'Nav2 not active — bt_navigator / lifecycle nodes down?')
            return False
        if err_holder[0] is not None:
            self._set(idx, FAILED, f'Nav2 wait raised: {err_holder[0]}')
            return False
        self._set(idx, PASSED, 'Nav2 lifecycle nodes active')
        return True

    def _test_flywheel_start(self, idx: int) -> bool:
        self._set(idx, RUNNING, 'calling /start_flywheel...')
        ok, msg = self.node.call_service_blocking(
            self.node.flywheel_start_client, 'start_flywheel'
        )
        self._set(idx, PASSED if ok else FAILED, msg)
        if ok:
            time.sleep(FLYWHEEL_SPINUP_S)  # let motors actually spin up
        return ok

    def _test_fire_ball(self, idx: int) -> bool:
        self._set(idx, RUNNING, 'calling /fire_ball (1 ball)...')
        ok, msg = self.node.call_service_blocking(
            self.node.fire_ball_client, 'fire_ball'
        )
        self._set(idx, PASSED if ok else FAILED, msg)
        if ok:
            time.sleep(FLYWHEEL_SPINDOWN_S)
        return ok

    def _test_flywheel_stop(self, idx: int) -> bool:
        self._set(idx, RUNNING, 'calling /stop_flywheel...')
        ok, msg = self.node.call_service_blocking(
            self.node.flywheel_stop_client, 'stop_flywheel'
        )
        self._set(idx, PASSED if ok else FAILED, msg)
        return ok

    # -- main entry -------------------------------------------------------------
    def run(self):
        log = self.node.get_logger()
        log.info('=' * 60)
        log.info('SYSTEM INTEGRATION TEST STARTING')
        log.info('=' * 60)

        # Tests are run in order. Hardware tests (flywheel/fire) always run
        # even if earlier non-hardware tests failed, but if /start_flywheel
        # fails we skip /fire_ball to avoid actuating the servo with motors off.
        steps: List[Callable[[int], bool]] = [
            self._test_node_init,
            self._test_tf_tree,
            self._test_marker_topic,
            self._test_marker_visible,
            self._test_nav2_active,
            self._test_flywheel_start,
            self._test_fire_ball,
            self._test_flywheel_stop,
        ]

        flywheel_started = False
        for i, step in enumerate(steps):
            try:
                if step is self._test_fire_ball and not flywheel_started:
                    self._set(i, SKIPPED, 'flywheel not running — refusing to fire')
                    continue
                ok = step(i)
                if step is self._test_flywheel_start:
                    flywheel_started = ok
            except Exception as e:  # noqa: BLE001
                self._set(i, FAILED, f'unhandled exception: {e}')
                log.error(f'Test {i} raised: {e}')

        # Safety: if flywheel started but stop failed (or test crashed earlier),
        # try one more stop call so we never leave motors running.
        if flywheel_started and self.results[7].status != PASSED:
            log.warn('Safety stop: forcing /stop_flywheel call.')
            self.node.call_service_blocking(
                self.node.flywheel_stop_client, 'stop_flywheel(safety)'
            )

        log.info('=' * 60)
        log.info('SYSTEM INTEGRATION TEST COMPLETE')
        log.info('=' * 60)
        for r in self.results:
            log.info(f'  [{r.status:7}] {r.name}  {r.message}')

        # Tell the GUI we're done so it can enable the Quit button.
        self.gui.mark_done(self.results)


# =============================================================================
# TKINTER GUI
# =============================================================================
class TestGUI:
    def __init__(self, on_quit: Callable[[], None]):
        self._on_quit = on_quit
        self.root = tk.Tk()
        self.root.title('CDE2310 Group 1 — System Integration Test')
        self.root.geometry('640x520')
        self.root.configure(bg='#1a1a1a')

        title = tk.Label(
            self.root,
            text='SYSTEM INTEGRATION TEST',
            font=('Helvetica', 16, 'bold'),
            fg='#ffffff', bg='#1a1a1a',
        )
        title.pack(pady=(16, 4))

        subtitle = tk.Label(
            self.root,
            text='Pre-flight check — no movement, fires 1 ball',
            font=('Helvetica', 10),
            fg='#aaaaaa', bg='#1a1a1a',
        )
        subtitle.pack(pady=(0, 12))

        # Test rows
        self.frame = tk.Frame(self.root, bg='#1a1a1a')
        self.frame.pack(fill='both', expand=True, padx=20)
        self._row_widgets = []  # list of (status_label, name_label, message_label)

        # Summary + quit
        self.summary = tk.Label(
            self.root, text='Running...', font=('Helvetica', 12, 'bold'),
            fg='#ffffff', bg='#1a1a1a',
        )
        self.summary.pack(pady=(12, 6))

        self.quit_btn = tk.Button(
            self.root, text='Quit', font=('Helvetica', 11),
            state='disabled', command=self._handle_quit,
            bg='#333333', fg='#ffffff', activebackground='#555555',
            relief='flat', padx=20, pady=6,
        )
        self.quit_btn.pack(pady=(0, 16))

        self.root.protocol('WM_DELETE_WINDOW', self._handle_quit)
        self._initialised_rows = False
        self._done = False

    def _ensure_rows(self, results: List[TestResult]):
        if self._initialised_rows:
            return
        for _ in results:
            row = tk.Frame(self.frame, bg='#1a1a1a')
            row.pack(fill='x', pady=3)
            status_lbl = tk.Label(
                row, text='[ ]', width=10, anchor='w',
                font=('Courier', 11, 'bold'),
                fg=STATUS_COLORS[PENDING], bg='#1a1a1a',
            )
            status_lbl.pack(side='left')
            name_lbl = tk.Label(
                row, text='', anchor='w',
                font=('Helvetica', 11),
                fg='#ffffff', bg='#1a1a1a', width=32,
            )
            name_lbl.pack(side='left')
            msg_lbl = tk.Label(
                row, text='', anchor='w',
                font=('Helvetica', 9),
                fg='#aaaaaa', bg='#1a1a1a',
            )
            msg_lbl.pack(side='left', fill='x', expand=True)
            self._row_widgets.append((status_lbl, name_lbl, msg_lbl))
        self._initialised_rows = True

    def refresh(self, results: List[TestResult]):
        # Called from worker thread — marshal onto Tk main loop.
        self.root.after(0, self._refresh_main, results)

    def _refresh_main(self, results: List[TestResult]):
        self._ensure_rows(results)
        for (status_lbl, name_lbl, msg_lbl), r in zip(self._row_widgets, results):
            status_lbl.config(text=f'[{r.status}]', fg=STATUS_COLORS[r.status])
            name_lbl.config(text=r.name)
            msg_lbl.config(text=r.message)

    def mark_done(self, results: List[TestResult]):
        self.root.after(0, self._mark_done_main, results)

    def _mark_done_main(self, results: List[TestResult]):
        self._done = True
        passed = sum(1 for r in results if r.status == PASSED)
        failed = sum(1 for r in results if r.status == FAILED)
        skipped = sum(1 for r in results if r.status == SKIPPED)
        total = len(results)
        if failed == 0 and skipped == 0:
            self.summary.config(text=f'ALL TESTS PASSED ({passed}/{total})', fg='#66bb6a')
        elif failed == 0:
            self.summary.config(
                text=f'PASSED {passed}/{total}  (skipped {skipped})', fg='#fdd835'
            )
        else:
            self.summary.config(
                text=f'FAILED — {passed} passed, {failed} failed, {skipped} skipped',
                fg='#ef5350',
            )
        self.quit_btn.config(state='normal')

    def _handle_quit(self):
        # Allow quit any time; if tests are still running, the executor
        # shutdown in main() will tear them down.
        self._on_quit()
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def mainloop(self):
        self.root.mainloop()


# =============================================================================
# MAIN
# =============================================================================
def main():
    rclpy.init()
    node = SystemTestNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin executor on a background thread so Tk owns the main thread.
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    shutdown_flag = {'fired': False}

    def shutdown():
        if shutdown_flag['fired']:
            return
        shutdown_flag['fired'] = True
        node.get_logger().info('Shutting down system test...')
        # Best-effort safety: stop flywheel if it might be running.
        try:
            if node.flywheel_stop_client.wait_for_service(timeout_sec=1.0):
                node.call_service_blocking(
                    node.flywheel_stop_client, 'stop_flywheel(shutdown)', timeout_s=2.0
                )
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    gui = TestGUI(on_quit=shutdown)
    runner = TestRunner(node, gui)

    # Run tests on a worker thread; GUI updates via root.after().
    worker = threading.Thread(target=runner.run, daemon=True)
    worker.start()

    try:
        gui.mainloop()
    finally:
        shutdown()


if __name__ == '__main__':
    main()
