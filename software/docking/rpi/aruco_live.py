import cv2
import cv2.aruco as aruco
import os
import argparse
import math
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    print("WARNING: rclpy not available. ROS 2 publishing disabled.")

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
if hasattr(aruco, "DetectorParameters_create"):
    params = aruco.DetectorParameters_create()
else:
    params = aruco.DetectorParameters()

detector = aruco.ArucoDetector(aruco_dict, params) if hasattr(aruco, "ArucoDetector") else None


def parse_source(value):
    return int(value) if value.isdigit() else value


def load_calibration(path):
    """Load camera matrix K (3x3) and distortion coefficients dist from a .npz file."""
    data = np.load(path)
    return data['K'].astype(np.float64), data['dist'].astype(np.float64)


def rotation_matrix_to_quaternion(R):
    """
    Convert a 3x3 rotation matrix to a unit quaternion (x, y, z, w).
    Uses the Shepperd trace method with four branches for numerical stability.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def estimate_marker_pose(corners_single, obj_pts, camera_matrix, dist_coeffs):
    """
    Estimate 6-DOF pose of a single ArUco marker using cv2.solvePnP.
    corners_single: shape (4, 2) — squeezed from detectMarkers output.
    Returns (rvec, tvec) or (None, None) on failure.
    """
    success, rvec, tvec = cv2.solvePnP(
        obj_pts, corners_single, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    return (rvec, tvec) if success else (None, None)


def build_and_publish_markers(ros2_node, ros2_pub, valid_ids, rvecs, tvecs, marker_size):
    """
    Build a visualization_msgs/MarkerArray from detected ArUco poses and publish it.
    Pose is in the camera frame (x right, y down, z into scene — OpenCV convention).
    """
    marker_array = MarkerArray()
    stamp = ros2_node.get_clock().now().to_msg()

    for i, marker_id in enumerate(valid_ids):
        R, _ = cv2.Rodrigues(rvecs[i])
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
        t = tvecs[i].flatten()

        m = Marker()
        m.header.frame_id = 'camera'
        m.header.stamp = stamp
        m.ns = 'aruco'
        m.id = int(marker_id)
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(t[0])
        m.pose.position.y = float(t[1])
        m.pose.position.z = float(t[2])
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        m.scale.x = marker_size
        m.scale.y = marker_size
        m.scale.z = 0.001
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.8
        marker_array.markers.append(m)

    ros2_pub.publish(marker_array)


parser = argparse.ArgumentParser(description="Live ArUco detection (GUI/headless + V4L2 output + ROS 2)")
parser.add_argument("--source", default="0", help="Camera source index or path (default: 0)")
parser.add_argument("--v4l2-out", default="", help="Optional V4L2 output device, e.g. /dev/video10")
parser.add_argument("--fps", type=float, default=30.0, help="Output FPS for V4L2 output (default: 30)")
parser.add_argument("--show", action="store_true", help="Force GUI window even if display vars are missing")
parser.add_argument("--marker-size", type=float, default=0.05,
                    help="Physical side length of ArUco markers in meters (default: 0.05)")
parser.add_argument("--calib", default="/home/ubuntu/calibration.npz",
                    help="Path to camera calibration .npz file (default: /home/ubuntu/calibration.npz)")
args = parser.parse_args()

has_gui = bool(args.show or os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))

# Load calibration and build marker object points (same layout as removed estimatePoseSingleMarkers)
camera_matrix, dist_coeffs = load_calibration(args.calib)
_half = args.marker_size / 2.0
MARKER_OBJ_PTS = np.array([
    [-_half,  _half, 0.0],  # top-left
    [ _half,  _half, 0.0],  # top-right
    [ _half, -_half, 0.0],  # bottom-right
    [-_half, -_half, 0.0],  # bottom-left
], dtype=np.float64)

# Initialize ROS 2 publisher (gracefully skipped if ROS 2 not sourced)
ros2_node = ros2_pub = None
if _ROS2_AVAILABLE:
    try:
        rclpy.init(args=None)
        ros2_node = Node('aruco_marker_publisher')
        ros2_pub = ros2_node.create_publisher(MarkerArray, '/aruco/markers', 10)
        print("ROS 2 publisher initialized on /aruco/markers")
    except Exception as e:
        print(f"WARNING: Could not initialize ROS 2 node: {e}")

cap = cv2.VideoCapture(parse_source(args.source), cv2.CAP_V4L2)
if not cap.isOpened():
    print(f"Cannot open camera source: {args.source}")
    exit(1)

if has_gui:
    print("Running ArUco detector. Press 'q' to quit.")
else:
    print("Running ArUco detector in headless mode (no display found). Press Ctrl+C to quit.")

writer = None
writer_open_failed = False

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        if args.v4l2_out and writer is None and not writer_open_failed:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            writer = cv2.VideoWriter(args.v4l2_out, cv2.CAP_V4L2, fourcc, args.fps, (w, h))
            if not writer.isOpened():
                writer_open_failed = True
                writer.release()
                writer = None
                print(f"Could not open V4L2 output device: {args.v4l2_out}")
            else:
                print(f"Streaming annotated frames to {args.v4l2_out}")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if detector is not None:
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=params)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, valid_ids = [], [], []
            for i, marker_id in enumerate(ids.flatten()):
                print(f"Marker ID: {marker_id}")
                if ros2_pub is not None:
                    rvec, tvec = estimate_marker_pose(
                        corners[i].squeeze(), MARKER_OBJ_PTS, camera_matrix, dist_coeffs)
                    if rvec is not None:
                        rvecs.append(rvec)
                        tvecs.append(tvec)
                        valid_ids.append(marker_id)
            if ros2_pub is not None and rvecs:
                build_and_publish_markers(ros2_node, ros2_pub, valid_ids, rvecs, tvecs, args.marker_size)
        else:
            cv2.putText(frame, f"No markers (rejected: {len(rejected)})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        if writer is not None:
            writer.write(frame)

        if has_gui:
            cv2.imshow("ArUco", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
except KeyboardInterrupt:
    pass
finally:
    cap.release()
    if writer is not None:
        writer.release()
    if has_gui:
        cv2.destroyAllWindows()
    if ros2_node is not None:
        ros2_node.destroy_node()
    if _ROS2_AVAILABLE and rclpy.ok():
        rclpy.shutdown()
