#!/usr/bin/env python3
"""Stage 1: your first ROS 2 node.

Drives the robot in a square using odometry, and stops if the laser sees
something too close in front.

  Publishes:  /cmd_vel  (geometry_msgs/Twist)        -> velocity commands
  Subscribes: /odom     (nav_msgs/Odometry)          -> where the robot thinks it is
              /scan     (sensor_msgs/LaserScan)      -> distances around the robot

Run (simulator must be running, ideally with nav:=false):
  ros2 run warehouse_runner square_driver
  ros2 run warehouse_runner square_driver --ros-args -p side_length:=1.5 -p loops:=2
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    """Convert a quaternion to a heading angle (radians) in the ground plane."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    """Keep an angle between -pi and +pi."""
    return math.atan2(math.sin(a), math.cos(a))


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')

        # Parameters can be changed from the command line or a launch file.
        self.side_length = self.declare_parameter('side_length', 1.0).value      # metres
        self.linear_speed = self.declare_parameter('linear_speed', 0.15).value   # m/s
        self.angular_speed = self.declare_parameter('angular_speed', 0.5).value  # rad/s
        self.safety_distance = self.declare_parameter('safety_distance', 0.35).value  # metres
        self.loops = self.declare_parameter('loops', 1).value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Sensor topics from Gazebo are "best effort", so we must subscribe with a matching QoS.
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.pose = None            # (x, y, yaw) from odometry
        self.front_distance = math.inf
        self.state = 'WAIT_FOR_ODOM'
        self.sides_done = 0
        self.segment_start = None   # (x, y) where the current side began
        self.target_yaw = None
        self.travelled = 0.0
        self.last_xy = None

        # The control loop: decide what to do 20 times per second.
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f'Square driver ready: side={self.side_length} m, loops={self.loops}. Waiting for /odom...')

    # ---------- callbacks: store the latest sensor data ----------
    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        if self.last_xy is not None:
            self.travelled += math.dist(self.last_xy, self.pose[:2])
        self.last_xy = self.pose[:2]

    def on_scan(self, msg):
        # Look at beams within +-30 degrees of straight ahead.
        closest = math.inf
        for i, r in enumerate(msg.ranges):
            angle = wrap_angle(msg.angle_min + i * msg.angle_increment)
            if abs(angle) < math.radians(30) and msg.range_min < r < msg.range_max:
                closest = min(closest, r)
        self.front_distance = closest

    # ---------- the state machine ----------
    def control_loop(self):
        cmd = Twist()

        if self.state == 'WAIT_FOR_ODOM':
            if self.pose is not None:
                self.get_logger().info(
                    f'Start pose x={self.pose[0]:.2f} y={self.pose[1]:.2f} '
                    f'yaw={math.degrees(self.pose[2]):.0f} deg')
                self.start_yaw = self.pose[2]
                self.start_side()

        elif self.state == 'FORWARD':
            dx = self.pose[0] - self.segment_start[0]
            dy = self.pose[1] - self.segment_start[1]
            remaining = self.side_length - math.hypot(dx, dy)
            if self.front_distance < self.safety_distance:
                self.get_logger().warn(
                    f'Obstacle {self.front_distance:.2f} m ahead - holding.', throttle_duration_sec=2.0)
                # cmd stays zero: robot stops until the path is clear
            elif remaining <= 0.0:
                self.start_turn()
            else:
                # Slow down for the last 20 cm so we don't overshoot.
                cmd.linear.x = self.linear_speed if remaining > 0.2 else max(0.05, self.linear_speed * remaining / 0.2)
                # Small steering correction keeps the side straight.
                cmd.angular.z = 1.5 * wrap_angle(self.side_heading() - self.pose[2])

        elif self.state == 'TURN':
            error = wrap_angle(self.target_yaw - self.pose[2])
            if abs(error) < math.radians(1.0):
                self.sides_done += 1
                if self.sides_done >= 4 * self.loops:
                    self.finish()
                else:
                    self.start_side()
            else:
                # Proportional control: turn faster when far from the target heading.
                cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 2.0 * error))
                if abs(cmd.angular.z) < 0.1:
                    cmd.angular.z = math.copysign(0.1, error)

        self.cmd_pub.publish(cmd)

    def start_side(self):
        self.segment_start = self.pose[:2]
        self.state = 'FORWARD'
        self.get_logger().info(f'Side {self.sides_done + 1}/{4 * self.loops}: driving forward')

    def side_heading(self):
        """Heading of the current side, measured from the start heading.

        Aiming each corner at start_yaw + k*90 deg stops small turn errors from adding up.
        (Try changing start_turn to use self.pose[2] + pi/2 instead and watch the drift.)
        """
        return wrap_angle(self.start_yaw + (self.sides_done % 4) * math.pi / 2)

    def start_turn(self):
        self.target_yaw = wrap_angle(self.start_yaw + ((self.sides_done + 1) % 4) * math.pi / 2)
        self.state = 'TURN'
        self.get_logger().info(f'Turning left to {math.degrees(self.target_yaw):.0f} deg')

    def finish(self):
        self.state = 'DONE'
        self.get_logger().info(
            f'Square complete. End pose x={self.pose[0]:.2f} y={self.pose[1]:.2f} '
            f'yaw={math.degrees(self.pose[2]):.0f} deg, odometry distance {self.travelled:.2f} m')
        raise SystemExit


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.cmd_pub.publish(Twist())  # always leave the robot stopped
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
