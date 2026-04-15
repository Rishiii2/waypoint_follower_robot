import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import yaml, math, time
from .pid_controller import PID
import tf_transformations
class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.timer = self.create_timer(0.1, self.loop)

        with open('config/waypoints.yaml') as f:
            self.waypoints = yaml.safe_load(f)['waypoints']

        self.wp_index = 0
        self.state = "ROTATE"

        self.x = self.y = self.yaw = 0.0

        # PID controllers
        self.linear_pid = PID(0.8, 0.0, 0.2)
        self.angular_pid = PID(2.0, 0.0, 0.3)

        self.last_time = time.time()
        def odom_cb(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y

    q = msg.pose.pose.orientation
    _, _, self.yaw = tf_transformations.euler_from_quaternion(
        [q.x, q.y, q.z, q.w])
        def loop(self):
    if self.wp_index >= len(self.waypoints):
        return

    now = time.time()
    dt = now - self.last_time
    self.last_time = now

    goal = self.waypoints[self.wp_index]

    dx = goal['x'] - self.x
    dy = goal['y'] - self.y

    distance = math.sqrt(dx**2 + dy**2)
    target_yaw = math.atan2(dy, dx)

    yaw_error = math.atan2(math.sin(target_yaw - self.yaw),
                           math.cos(target_yaw - self.yaw))

    cmd = Twist()
    if self.state == "ROTATE":
    if abs(yaw_error) > 0.05:
        cmd.angular.z = self.angular_pid.compute(yaw_error, dt)
    else:
        self.state = "MOVE"
    
    elif self.state == "MOVE":
    if distance > 0.1:
        cmd.linear.x = self.linear_pid.compute(distance, dt)
        cmd.angular.z = self.angular_pid.compute(yaw_error, dt)
    else:
        self.state = "STOP"
        elif self.state == "STOP":
    self.wp_index += 1
    self.state = "ROTATE"
    self.cmd_pub.publish(cmd)
