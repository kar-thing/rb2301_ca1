import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message

        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        self.last_scan = msg # Slices the 721 scan array to return only 36 scans. Feel free to edit

    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.

        ######################## MODIFY CODE HERE ########################
        ranges = np.array(self.last_scan.ranges)
    # Replace inf (no obstacle in range) / nan with a large "clear" distance
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0, neginf=0.0)

        angle_min = self.last_scan.angle_min
        angle_increment = self.last_scan.angle_increment
        angles = angle_min + np.arange(len(ranges)) * angle_increment

        # Define angular sectors (radians). 0 rad = straight ahead.
        #front_mask = np.abs(angles) < np.deg2rad(20)
        #left_mask = (angles >= np.deg2rad(20)) & (angles < np.deg2rad(90))
        #right_mask = (angles <= -np.deg2rad(20)) & (angles > -np.deg2rad(90))

        #Robot's physical forward direction is approximately -90 degrees

        front_mask = ((angles >= np.deg2rad(150)) |(angles <= np.deg2rad(-150)))
        # Physical LEFT of robot
        left_mask = ((angles > np.deg2rad(-150)) & (angles < np.deg2rad(-70)))

# Physical RIGHT of robot
        right_mask = ((angles > np.deg2rad(70)) & (angles < np.deg2rad(150)))
        
        front_min = np.min(ranges[front_mask]) if np.any(front_mask) else 10.0
        left_min = np.min(ranges[left_mask]) if np.any(left_mask) else 10.0
        right_min = np.min(ranges[right_mask]) if np.any(right_mask) else 10.0

        closest_index = np.argmin(ranges)
        closest_distance = ranges[closest_index]
        closest_angle = np.rad2deg(angles[closest_index])

        self.get_logger().info(
            f"Closest object = {closest_distance:.2f} m "
            f"at angle {closest_angle:.1f} degrees"
        )

        self.get_logger().debug(
        f"front={front_min:.2f} left={left_min:.2f} right={right_min:.2f}"
        )

        self.get_logger().info(
        f"front={front_min:.2f} left={left_min:.2f} right={right_min:.2f}"
        )

        safe_distance = 0.5 # meters — tune to your robot/environment

        if front_min < safe_distance:
        # Obstacle ahead: stop forward motion, turn toward the more open side
            if left_min > right_min:
                self.move_2D(0.0, 0.2, 0.0) # move left
            else:
                self.move_2D(0.0, 0.2, 0.0) # move right
        else:
            # Clear ahead: drive forward
            self.move_2D(0.2, 0.0, 0.0)
            ######################## MODIFY CODE HERE ########################


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()