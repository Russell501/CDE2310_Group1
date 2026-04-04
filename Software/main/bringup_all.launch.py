# bringup_all.launch.py
# Unified launch file for CDE2310 Group 1 TurtleBot3 Warehouse Mission
#
# Startup order (event-driven, no blind timers):
#   1. Cartographer  — starts immediately
#   2. Nav2          — starts when Cartographer prints CARTOGRAPHER_READY
#   3. explore_lite  — starts when Nav2 prints NAV2_READY
#
# To find the right trigger strings:
#   Run each subsystem manually and note the log line printed when it is ready.
#   Only a unique substring is needed — not the full line.

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch_ros.actions import Node


# =============================================================================
# TRIGGER STRINGS
# =============================================================================

CARTOGRAPHER_READY = 'Trying to create a map of size'

NAV2_READY = 'Creating bond timer'

EXPLORE_READY = 'Connected to move_base nav2 server' 


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

    fsm_node = Node(
        package='auto_nav',
        executable='mission_controller',
        name='ultimate_mission_controller',
        output='screen',
    )

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

    # Step 3: Nav2 ready → launch explore_lite
    on_nav2_ready = RegisterEventHandler(
        OnProcessIO(
            on_stdout=_make_trigger(
                NAV2_READY,
                explore_cmd, 'explore_process',
                'Nav2 ready — launching explore_lite...',
                launched,
            ),
        )
    )

    # Step 4: explore_lite ready → launch FSM
    def on_explore_ready_cb(event):
        if 'fsm_node' in launched:
            return []
        text = (event.text.decode('utf-8', errors='replace')
                if isinstance(event.text, bytes) else str(event.text))
        if EXPLORE_READY in text:
            launched.add('fsm_node')
            return [
                LogInfo(msg='explore_lite ready — launching mission FSM...'),
                fsm_node,
            ]
        return []

    on_explore_ready = RegisterEventHandler(
        OnProcessIO(on_stdout=on_explore_ready_cb)
    )

    return LaunchDescription([
        cartographer,
        on_cartographer_ready,
        on_nav2_ready,
        on_explore_ready,
    ])
