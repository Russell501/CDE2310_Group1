#!/usr/bin/env python3
"""Launch all mission processes: local ROS2 nodes + remote TurtleBot scripts."""

import subprocess
import signal
import sys
import time

REMOTE_USER = "ubuntu"
REMOTE_HOST = "172.20.10.12"
REMOTE_PASS = "!qaz"

SSH_CMD = ["sshpass", "-p", REMOTE_PASS, "ssh", "-o", "StrictHostKeyChecking=no", f"{REMOTE_USER}@{REMOTE_HOST}"]

processes = []

def cleanup(sig=None, frame=None):
    print("\nShutting down all processes...")
    for name, proc in reversed(processes):
        if proc.poll() is None:
            print(f"  Stopping {name}...")
            proc.terminate()
    for name, proc in processes:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print(f"  Force killing {name}...")
            proc.kill()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

def launch(name, cmd, **kwargs):
    print(f"[+] Launching: {name}")
    proc = subprocess.Popen(cmd, **kwargs)
    processes.append((name, proc))
    return proc

# Remote processes (TurtleBot)
launch("remote: rosbu",           SSH_CMD + ["rosbu"])
launch("remote: aruco_live.py",   SSH_CMD + ["python3 ~/aruco_live.py"])
launch("remote: ball_launcher.py", SSH_CMD + ["python3 ~/ball_launcher.py"])

# Local processes
launch("local: bringup_all",  ["ros2", "launch", "auto_nav", "bringup_all.launch.py"])

# Give bringup time to start before GUI
time.sleep(3)

launch("local: mission_gui",  ["ros2", "run", "auto_nav", "mission_gui"])

print("\nAll processes launched. Press Ctrl+C to stop all.\n")

# Wait for any process to exit
while True:
    for name, proc in processes:
        ret = proc.poll()
        if ret is not None:
            print(f"[!] {name} exited with code {ret}")
    time.sleep(1)
