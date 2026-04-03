# bringup_all.launch.py
# Unified launch file for CDE2310 Group 1 TurtleBot3 Warehouse Mission
#
# Startup order (event-driven, no blind timers):
#   1. Cartographer       — starts immediately
#   2. Nav2               — starts when Cartographer prints CARTOGRAPHER_READY
#   3. explore_lite  \
#      aruco_dock_deferred — both start when Nav2 prints NAV2_READY
#   4. /start_docking pub — fires when explore_lite prints EXPLORE_DONE
#                           (wakes aruco_dock_deferred into Phase 2)
#
# aruco_dock_deferred phases:
#   Phase 1 (scouting) — runs silently alongside explore_lite, saving ArUco
#                         marker poses to /tmp/aruco_dock_poses.yaml as the
#                         robot passes each marker during exploration.
#   Phase 2 (docking)  — triggered by /start_docking; navigates to marker 0
#                         (static dock), undocks, then navigates to marker 1
#                         (moving dock) and waits for launch signal.
#
# To find the right trigger strings:
#   Run each subsystem manually and note the log line printed when it is ready.
#   Only a unique substring is needed — not the full line.

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessIO


# =============================================================================
# TRIGGER STRINGS
# =============================================================================

CARTOGRAPHER_READY = 'Trying to create a map of size'

NAV2_READY = 'Creating bond timer'

EXPLORE_READY = 'Connected to move_base nav2 server'

# Printed by explore_lite when all frontiers are exhausted.
# Verify against your explore_lite version by running it manually:
#   common candidates: 'Exploration stopped.' / 'No frontiers found'
EXPLORE_DONE = 'Exploration stopped.'


# =============================================================================
# PATH TO DOCKING SCRIPT
# =============================================================================

# Set this to the absolute path of aruco_dock_deferred.py on your laptop.
ARUCO_DOCK_SCRIPT = '/absolute/path/to/aruco_dock_deferred.py'


# =============================================================================
# HELPERS
# =============================================================================

def _make_trigger(trigger_str, next_cmd, next_name, log_msg, launched):
    """Return an OnProcessIO stdout callback that fires once when trigger matches."""
    def callback(event):
        if trigger_str == 'TRIGGER_PLACEHOLDER':
            return []
        if next_name in launched:
            return []
        text = (event.text.decode('utf-8', errors='replace')
                if isinstance(event.text, bytes) else str(event.text))
        if trigger_str in text:
            launched.add(next_name)
            return [
                LogInfo(msg=log_msg),
                ExecuteProcess(cmd=next_cmd, output='screen', name=next_name),
            ]
        return []
    return callback


def _make_dual_trigger(trigger_str, cmd_a, name_a, cmd_b, name_b, log_msg, launched):
    """
    OnProcessIO callback that launches TWO processes simultaneously on trigger.
    Used to start explore_lite and aruco_dock_deferred at the same moment.
    """
    def callback(event):
        if name_a in launched:
            return []
        text = (event.text.decode('utf-8', errors='replace')
                if isinstance(event.text, bytes) else str(event.text))
        if trigger_str in text:
            launched.add(name_a)
            launched.add(name_b)
            return [
                LogInfo(msg=log_msg),
                ExecuteProcess(cmd=cmd_a, output='screen', name=name_a),
                ExecuteProcess(cmd=cmd_b, output='screen', name=name_b),
            ]
        return []
    return callback


# =============================================================================
# LAUNCH DESCRIPTION
# =============================================================================

def generate_launch_description():

    launched = set()   # Tracks which processes have already been started

    # -------------------------------------------------------------------------
    # Process definitions
    # -------------------------------------------------------------------------

    cartographer = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            'turtlebot3_cartographer', 'cartographer.launch.py',
            'use_sim_time:=false',
        ],
        output='screen',
        name='cartographer_process',
    )

    nav2_cmd = [
        'ros2', 'launch',
        'nav2_bringup', 'navigation_launch.py',
        'params_file:=/opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml',
        'use_sim_time:=false',
    ]

    explore_cmd = [
        'ros2', 'launch',
        'explore_lite', 'explore.launch.py',
        # Uncomment and set path if using a custom params file:
        # 'params_file:=/absolute/path/to/explore_params.yaml',
    ]

    # aruco_dock_deferred.py — Phase 1 runs silently during exploration,
    # Phase 2 is triggered by /start_docking below.
    aruco_dock_cmd = [
        'python3', ARUCO_DOCK_SCRIPT,
    ]

    # One-shot publisher: wakes the docking script into Phase 2.
    # Published 3 times in quick succession to survive any transient drop.
    start_docking_cmd = [
        'ros2', 'topic', 'pub',
        '--times', '3',
        '/start_docking',
        'std_msgs/msg/Bool',
        '{data: true}',
    ]

    # -------------------------------------------------------------------------
    # Event chain
    # -------------------------------------------------------------------------

    # Step 2: Cartographer ready → launch Nav2
    on_cartographer_ready = RegisterEventHandler(
        OnProcessIO(
            target_action=cartographer,
            on_stdout=_make_trigger(
                CARTOGRAPHER_READY,
                nav2_cmd, 'nav2_process',
                'Cartographer ready — launching Nav2...',
                launched,
            ),
        )
    )

    # Step 3: Nav2 ready → launch explore_lite AND aruco_dock_deferred together.
    #
    # aruco_dock_deferred enters Phase 1 immediately: it subscribes to
    # /aruco/markers and silently saves dock poses as the robot explores.
    # It blocks on dock_event.wait() until /start_docking fires in Step 4.
    #
    # OnProcessIO without target_action watches all processes — necessary
    # because nav2_process is created dynamically in Step 2.
    on_nav2_ready = RegisterEventHandler(
        OnProcessIO(
            on_stdout=_make_dual_trigger(
                NAV2_READY,
                explore_cmd,     'explore_process',
                aruco_dock_cmd,  'aruco_dock_process',
                'Nav2 ready — launching explore_lite and aruco_dock_deferred...',
                launched,
            ),
        )
    )

    # Step 4: explore_lite done → publish /start_docking.
    #
    # This wakes aruco_dock_deferred out of Phase 1 into Phase 2.
    # The docking script verifies both ArUco poses are saved, then runs:
    #   → Navigate + fine-approach to marker 0 (static dock)
    #   → Reverse UNDOCK_DISTANCE to clear the dock
    #   → Navigate + fine-approach to marker 1 (moving dock)
    #   → Wait for launch signal (ArUco re-detection)
    #
    # NOTE: Confirm EXPLORE_DONE matches what your explore_lite version
    # actually prints when exploration finishes. Run it manually and check.
    on_explore_done = RegisterEventHandler(
        OnProcessIO(
            on_stdout=_make_trigger(
                EXPLORE_DONE,
                start_docking_cmd, 'start_docking_process',
                'Exploration complete — triggering docking sequence...',
                launched,
            ),
        )
    )

    return LaunchDescription([
        cartographer,
        on_cartographer_ready,
        on_nav2_ready,
        on_explore_done,
    ])
