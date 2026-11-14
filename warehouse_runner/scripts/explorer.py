#!/usr/bin/env python3
"""Stage 3: drive an exploration route so slam_toolbox can build the map, then save it.

The route covers the outer corridor loop and the centre aisle. With a 3.5 m laser that is
enough to see every wall and shelf. It is a simple go-to-point controller:
  - turn towards the next waypoint if facing the wrong way,
  - otherwise drive forward and steer proportionally,
  - hold still if the laser sees something closer than safety_distance ahead.
The robot's position comes from TF (map -> base_footprint), which is SLAM's corrected pose.

  ros2 run warehouse_runner explorer --ros-args -p use_sim_time:=true
  ros2 run warehouse_runner explorer --ros-args -p use_sim_time:=true -p save:=false
Saved maps go to ~/ros2_ws/maps/ (slam_map.yaml = latest, plus a timestamped copy).
"""
import math
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

# Outer loop (counter-clockwise from HOME), then up the centre aisle to station C.
ROUTE = [(3.7, -3.8), (3.7, 0.0), (3.7, 3.8), (0.0, 3.8), (-3.7, 3.8), (-3.7, 0.0),
         (-3.7, -3.8), (0.0, -3.8), (0.0, 0.0), (0.0, 3.8)]
MAP_DIR = Path.home() / 'ros2_ws' / 'maps'


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.speed = self.declare_parameter('speed', 0.2).value
        self.safety = self.declare_parameter('safety_distance', 0.3).value
        self.save = self.declare_parameter('save', True).value
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/explorer/status', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.front = math.inf
        self.index = 0
        self.blocked_since = None
        self.travelled = 0.0
        self.last = None
        self.started = None
        self.done = False
        self.create_timer(0.05, self.loop)
        self.get_logger().info(f'Explorer ready: {len(ROUTE)} waypoints. Waiting for the map frame...')

    def on_scan(self, scan):
        best = math.inf
        for i, r in enumerate(scan.ranges):
            a = wrap(scan.angle_min + i * scan.angle_increment)
            if abs(a) < math.radians(25) and scan.range_min < r < scan.range_max:
                best = min(best, r)
        self.front = best

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def say(self, text):
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def loop(self):
        if self.done:
            return
        p = self.pose()
        if p is None:
            return
        if self.started is None:
            self.started = time.monotonic()
            self.say(f'Exploring from ({p[0]:.2f}, {p[1]:.2f}). Waypoint 1/{len(ROUTE)}')
        if self.last:
            self.travelled += math.dist(self.last, p[:2])
        self.last = p[:2]

        tx, ty = ROUTE[self.index]
        dist = math.hypot(tx - p[0], ty - p[1])
        cmd = Twist()
        if dist < 0.15:
            self.index += 1
            if self.index >= len(ROUTE):
                self.cmd.publish(Twist())
                self.finish()
                return
            self.say(f'Waypoint {self.index + 1}/{len(ROUTE)}: {ROUTE[self.index]}')
            return
        err = wrap(math.atan2(ty - p[1], tx - p[0]) - p[2])
        if abs(err) > 0.35:
            cmd.angular.z = math.copysign(min(0.8, max(0.25, abs(err) * 1.5)), err)  # turn on the spot
        elif self.front < self.safety:
            if self.blocked_since is None:
                self.blocked_since = time.monotonic()
                self.get_logger().warn(f'Obstacle {self.front:.2f} m ahead - waiting')
            if time.monotonic() - self.blocked_since > 8:   # give up on this waypoint
                self.get_logger().warn('Still blocked after 8 s - skipping waypoint')
                self.index = min(self.index + 1, len(ROUTE) - 1)
                self.blocked_since = None
        else:
            self.blocked_since = None
            cmd.linear.x = min(self.speed, 0.4 * dist + 0.05)
            cmd.angular.z = max(-0.8, min(0.8, 2.0 * err))
        self.cmd.publish(cmd)

    def finish(self):
        self.done = True
        took = time.monotonic() - self.started
        self.say(f'Exploration complete: {self.travelled:.1f} m in {took:.0f} s.')
        if not self.save:
            raise SystemExit
        time.sleep(3.0)  # let slam_toolbox publish a final map update
        MAP_DIR.mkdir(parents=True, exist_ok=True)
        (MAP_DIR / 'slam_map.yaml').unlink(missing_ok=True)   # so "saved" below is never a stale file
        stamp = time.strftime('%Y%m%d-%H%M%S')
        for name in ['slam_map', 'slam_map_' + stamp]:
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(MAP_DIR / name),
                            '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=10.0'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
        ok = (MAP_DIR / 'slam_map.yaml').exists()
        self.say(f'Map saved to {MAP_DIR / "slam_map.yaml"}' if ok else 'Map save FAILED - is slam_toolbox running?')
        raise SystemExit


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.cmd.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
