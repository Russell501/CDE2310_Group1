import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf2_ros

class MazeCoverageNode(Node):
    def __init__(self):
        super().__init__('maze_coverage')
        
        # Subscribe to the known map
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Nav2 action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        
        # TF for getting robot solposition
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.map_data = None
        self.visited = None
        self.resolution = None
        self.origin = None
        self.navigating = False
        self.visit_radius = 0.3  # meters — mark cells within this as visited
        self.goal_spacing = 0.5  # meters — grid spacing for coverage goals
        
        # Periodically check and send next goal
        self.timer = self.create_timer(1.0, self.coverage_tick)
        self.get_logger().info('Maze coverage node started')

    def map_callback(self, msg):
        w, h = msg.info.width, msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.map_data = np.array(msg.data).reshape((h, w))
        
        if self.visited is None:
            self.visited = np.zeros_like(self.map_data, dtype=bool)
            # Mark walls and unknown as "visited" (don't need to go there)
            self.visited[self.map_data != 0] = True
            self.get_logger().info(
                f'Map received: {w}x{h}, '
                f'{np.sum(~self.visited)} free cells to cover')

    def get_robot_position(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link',
                                                 rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def world_to_grid(self, x, y):
        col = int((x - self.origin.position.x) / self.resolution)
        row = int((y - self.origin.position.y) / self.resolution)
        return row, col

    def grid_to_world(self, row, col):
        x = self.origin.position.x + (col + 0.5) * self.resolution
        y = self.origin.position.y + (row + 0.5) * self.resolution
        return x, y

    def mark_visited(self, robot_x, robot_y):
        """Mark cells near robot as visited."""
        r, c = self.world_to_grid(robot_x, robot_y)
        radius_cells = int(self.visit_radius / self.resolution)
        h, w = self.visited.shape
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w:
                    if dr*dr + dc*dc <= radius_cells*radius_cells:
                        self.visited[nr, nc] = True

    def find_nearest_unvisited(self, robot_x, robot_y):
        """BFS from robot position to find closest unvisited free cell."""
        from collections import deque
        r, c = self.world_to_grid(robot_x, robot_y)
        h, w = self.visited.shape
        
        # Subsample to avoid picking goals too close together
        step = max(1, int(self.goal_spacing / self.resolution))
        
        seen = set()
        queue = deque([(r, c)])
        seen.add((r, c))
        
        while queue:
            cr, cc = queue.popleft()
            # Check if this is an unvisited free cell at grid spacing
            if (not self.visited[cr, cc] and 
                cr % step == 0 and cc % step == 0):
                return self.grid_to_world(cr, cc)
            
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr, nc = cr + dr, cc + dc
                if (0 <= nr < h and 0 <= nc < w and 
                    (nr, nc) not in seen and
                    self.map_data[nr, nc] == 0):  # only traverse free space
                    seen.add((nr, nc))
                    queue.append((nr, nc))
        return None  # all covered!

    def coverage_tick(self):
        if self.map_data is None or self.navigating:
            return
        
        pos = self.get_robot_position()
        if pos is None:
            return
        
        # Mark current area as visited
        self.mark_visited(*pos)
        
        # How much is left?
        total_free = np.sum(self.map_data == 0)
        visited_free = np.sum(self.visited & (self.map_data == 0))
        pct = visited_free / total_free * 100 if total_free > 0 else 100
        
        # Find next unvisited cell
        target = self.find_nearest_unvisited(*pos)
        if target is None:
            self.get_logger().info(f'Coverage complete! {pct:.1f}%')
            self.timer.cancel()
            return
        
        self.get_logger().info(
            f'Coverage: {pct:.1f}% — navigating to ({target[0]:.2f}, {target[1]:.2f})')
        self.send_goal(*target)

    def send_goal(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        
        self.navigating = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_accepted_callback)

    def goal_accepted_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.navigating = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future):
        self.navigating = False  # triggers next goal on next tick


def main():
    rclpy.init()
    node = MazeCoverageNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()