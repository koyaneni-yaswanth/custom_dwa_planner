#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

class CustomDWAPlanner(Node):
    def __init__(self):
        super().__init__('custom_dwa_planner')
        self.get_logger().info("Custom DWA Planner node started.")

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Parameters
        self.declare_parameter('max_vel_x', 0.5)
        self.declare_parameter('min_vel_x', 0.05)
        self.declare_parameter('max_vel_theta', 4.0)
        self.declare_parameter('max_acc_x', 3.5)
        self.declare_parameter('max_acc_theta', 4.5)
        self.declare_parameter('sim_time', 2.0)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('robot_radius', 0.10)
        self.declare_parameter('heading_weight', 1.0)
        self.declare_parameter('velocity_weight', 0.3)
        self.declare_parameter('obstacle_weight', 0.5)

        # State variables
        self.current_pose = None
        self.current_yaw = 0.0
        self.laser_ranges = []
        self.goal = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg: LaserScan):
        self.laser_ranges = msg.ranges

    def goal_callback(self, msg: PoseStamped):
        self.goal = msg.pose.position
        self.get_logger().info(f"New goal received: x={self.goal.x:.2f}, y={self.goal.y:.2f}")

    def control_loop(self):
        if self.current_pose is None or self.goal is None or not self.laser_ranges:
            return

        best_cost = float('inf')
        best_twist = Twist()
        best_marker_array = MarkerArray()

        # Get parameters
        max_vel_x = self.get_parameter('max_vel_x').value
        max_vel_theta = self.get_parameter('max_vel_theta').value
        sim_time = self.get_parameter('sim_time').value
        dt = self.get_parameter('dt').value

        for v in [i * 0.1 for i in range(0, int(max_vel_x * 10) + 1)]:
            for w in [i * 0.2 - max_vel_theta for i in range(0, int(max_vel_theta * 10) + 1, 4)]:
                cost, traj_marker = self.evaluate_trajectory(v, w, sim_time, dt)
                if cost < best_cost:
                    best_cost = cost
                    best_twist.linear.x = v
                    best_twist.angular.z = w
                    best_marker_array.markers = [traj_marker]

        # Publish chosen velocity
        self.cmd_pub.publish(best_twist)
        self.traj_pub.publish(best_marker_array)

    def evaluate_trajectory(self, v, w, sim_time, dt):
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        theta = self.current_yaw
        total_time = 0.0

        points = []
        collision = False

        while total_time < sim_time:
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt
            total_time += dt
            points.append((x, y))

            # Basic obstacle check
            for i, r in enumerate(self.laser_ranges):
                if math.isinf(r) or math.isnan(r):
                    continue
                angle = math.radians(i)
                ox = self.current_pose.position.x + r * math.cos(angle)
                oy = self.current_pose.position.y + r * math.sin(angle)
                dist = math.hypot(ox - x, oy - y)
                if dist < self.get_parameter('robot_radius').value * 0.5:
                    collision = True
                    break
            if collision:
                break

        # Cost functions
        if collision:
            return 1e6, self.make_marker(points, (1.0, 0.0, 0.0))

        goal_dist = math.hypot(self.goal.x - x, self.goal.y - y)
        heading_cost = goal_dist
        velocity_cost = (self.get_parameter('max_vel_x').value - v)
        total_cost = (
            self.get_parameter('heading_weight').value * heading_cost +
            self.get_parameter('velocity_weight').value * velocity_cost
        )

        return total_cost, self.make_marker(points, (0.0, 1.0, 0.0))

    def make_marker(self, points, color):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.points = []

        for (x, y) in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.01
            marker.points.append(p)

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = CustomDWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

