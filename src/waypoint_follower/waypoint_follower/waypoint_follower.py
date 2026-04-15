import rclpy
from rclpy.node import Node

import math
import yaml

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class State:
    ROTATE = 0
    MOVE = 1
    STOP = 2
    NEXT = 3


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def clamp(v, min_v, max_v):
    return max(min(v, max_v), min_v)


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        # Publisher & Subscriber
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Parameter (full path required)
        self.declare_parameter('waypoint_file', '')
        file_path = self.get_parameter('waypoint_file').value

        self.load_waypoints(file_path)

        # State variables
        self.state = State.ROTATE
        self.wp_index = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Timer loop
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("Waypoint follower started")

    def load_waypoints(self, file_path):
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)

        self.waypoints = data['waypoints']
        self.pos_tol = data['tolerances']['position']
        self.angle_tol = data['tolerances']['angle']

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def loop(self):
        if self.wp_index >= len(self.waypoints):
            self.stop()
            return

        goal = self.waypoints[self.wp_index]

        dx = goal['x'] - self.x
        dy = goal['y'] - self.y

        dist = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        error = self.normalize(target_angle - self.yaw)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        # STATE MACHINE
        if self.state == State.ROTATE:
            cmd.twist.angular.z = self.rotate(error)

            if abs(error) < self.angle_tol:
                self.state = State.MOVE

        elif self.state == State.MOVE:
            cmd.twist.linear.x = self.move(dist)
            cmd.twist.angular.z = self.rotate(error)

            if dist < self.pos_tol:
                self.state = State.STOP

        elif self.state == State.STOP:
            self.stop()
            self.get_logger().info(f"Reached waypoint {self.wp_index}")
            self.state = State.NEXT

        elif self.state == State.NEXT:
            self.wp_index += 1
            self.state = State.ROTATE

        self.pub.publish(cmd)

    def rotate(self, error):
        return clamp(1.5 * error, -1.0, 1.0)

    def move(self, dist):
        return clamp(0.8 * dist, 0.0, 0.5)

    def stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(cmd)

    def normalize(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
