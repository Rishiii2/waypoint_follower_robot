from sensor_msgs.msg import LaserScan

class ObstacleAvoidance:
    def __init__(self):
        self.min_dist = float('inf')

    def scan_callback(self, msg):
        self.min_dist = min(msg.ranges)

    def avoid(self, cmd):
        if self.min_dist < 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # turn
        return cmd
