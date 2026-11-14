#!/usr/bin/env python3
"""Stage 2: understand what the robot sees (LaserScan) and where things are (TF frames).

Every laser scan:
  1. Splits the 360 beams into FRONT / LEFT / BACK / RIGHT and finds the closest wall in each.
  2. Takes the single closest point. It is measured in the laser's own frame (base_scan), so
     it uses TF to move that point into the odom frame (warehouse coordinates).
  3. Draws it in RViz: a red ball on the obstacle, a line from the laser to it, and a label.
It also asks TF "where is the robot?" twice a second and publishes the answer as a trail.

  Subscribes: /scan                (sensor_msgs/LaserScan)
  Uses TF:    odom -> base_footprint -> base_link -> base_scan
  Publishes:  /stage2/markers      (visualization_msgs/MarkerArray)  -> RViz
              /stage2/trail        (nav_msgs/Path)                    -> RViz
              /stage2/sectors      (std_msgs/String)                  -> ros2 topic echo

Run:  ros2 run warehouse_runner scan_inspector --ros-args -p use_sim_time:=true
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

SECTORS = {'FRONT': 0, 'LEFT': 90, 'BACK': 180, 'RIGHT': -90}   # centre angle, degrees


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform_point(tf, x, y):
    """Move point (x, y) from the child frame into the parent frame of a 2D transform.

    This is the core TF idea: rotate by the frame's heading, then shift by its position.
    """
    t = tf.transform.translation
    yaw = yaw_from_quaternion(tf.transform.rotation)
    return (t.x + math.cos(yaw) * x - math.sin(yaw) * y,
            t.y + math.sin(yaw) * x + math.cos(yaw) * y)


class ScanInspector(Node):
    def __init__(self):
        super().__init__('scan_inspector')
        self.fixed_frame = self.declare_parameter('fixed_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value

        # The TF buffer collects every transform published on /tf and /tf_static, and can
        # chain them: odom->base_footprint + base_footprint->base_link + base_link->base_scan.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, '/stage2/markers', 10)
        self.trail_pub = self.create_publisher(Path, '/stage2/trail', 10)
        self.sector_pub = self.create_publisher(String, '/stage2/sectors', 10)

        self.trail = Path()
        self.trail.header.frame_id = self.fixed_frame
        self.create_timer(0.5, self.update_trail)
        self.create_timer(4.0, self.print_tf_tree_once)
        self.tree_printed = False
        self.scan_info_printed = False
        self.get_logger().info('Scan inspector started. Open RViz to see markers and trail.')

    # ---------- laser ----------
    def on_scan(self, scan):
        if not self.scan_info_printed:
            self.scan_info_printed = True
            self.get_logger().info(
                f'LaserScan: frame="{scan.header.frame_id}", {len(scan.ranges)} beams, '
                f'{math.degrees(scan.angle_min):.0f} to {math.degrees(scan.angle_max):.0f} deg, '
                f'step {math.degrees(scan.angle_increment):.1f} deg, '
                f'range {scan.range_min:.2f}-{scan.range_max:.2f} m')

        sector_min = {name: math.inf for name in SECTORS}
        closest = (math.inf, 0.0)  # (distance, angle in radians)
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max):
                continue  # inf / nan / too close = no reading for this beam
            angle = math.atan2(math.sin(scan.angle_min + i * scan.angle_increment),
                               math.cos(scan.angle_min + i * scan.angle_increment))
            deg = math.degrees(angle)
            for name, centre in SECTORS.items():
                if abs(math.degrees(math.atan2(math.sin(math.radians(deg - centre)),
                                               math.cos(math.radians(deg - centre))))) <= 45:
                    sector_min[name] = min(sector_min[name], r)
            if r < closest[0]:
                closest = (r, angle)

        text = ' | '.join(f'{n} {"-" if math.isinf(d) else f"{d:.2f} m"}' for n, d in sector_min.items())
        self.sector_pub.publish(String(data=text))
        self.get_logger().info(text, throttle_duration_sec=2.0)

        if math.isinf(closest[0]):
            return
        # The point in the LASER's frame: x forward, y left.
        px, py = closest[0] * math.cos(closest[1]), closest[0] * math.sin(closest[1])
        try:
            # "Where was base_scan inside odom at the moment this scan was taken?"
            # (No timeout: waiting here would block the same thread that receives /tf.)
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, scan.header.frame_id,
                                                 Time.from_msg(scan.header.stamp))
            self.stamped_ok = getattr(self, 'stamped_ok', 0) + 1
        except TransformException:
            try:
                # The scan can be a few ms newer than the last /tf. Fall back to the latest.
                tf = self.tf_buffer.lookup_transform(self.fixed_frame, scan.header.frame_id, Time())
                self.fallbacks = getattr(self, 'fallbacks', 0) + 1
            except TransformException as e:
                self.get_logger().warn(f'TF not ready: {e}', throttle_duration_sec=5.0)
                return
        wx, wy = transform_point(tf, px, py)
        laser = tf.transform.translation
        self.publish_markers(scan.header.stamp, (laser.x, laser.y), (wx, wy), closest)

    def publish_markers(self, stamp, laser_xy, obstacle_xy, closest):
        def marker(mid, mtype):
            m = Marker()
            m.header.frame_id = self.fixed_frame
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = 'stage2', mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.a = 1.0
            return m

        ball = marker(0, Marker.SPHERE)
        ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = obstacle_xy[0], obstacle_xy[1], 0.15
        ball.scale.x = ball.scale.y = ball.scale.z = 0.15
        ball.color.r, ball.color.g, ball.color.b = 1.0, 0.2, 0.2

        line = marker(1, Marker.LINE_STRIP)
        line.scale.x = 0.02
        line.color.r, line.color.g, line.color.b = 1.0, 0.8, 0.2
        line.points = [Point(x=laser_xy[0], y=laser_xy[1], z=0.17), Point(x=obstacle_xy[0], y=obstacle_xy[1], z=0.15)]

        label = marker(2, Marker.TEXT_VIEW_FACING)
        label.pose.position.x, label.pose.position.y, label.pose.position.z = laser_xy[0], laser_xy[1], 0.6
        label.scale.z = 0.18
        label.color.r = label.color.g = label.color.b = 1.0
        label.text = (f'closest {closest[0]:.2f} m at {math.degrees(closest[1]):.0f} deg (laser frame)\n'
                      f'= ({obstacle_xy[0]:.2f}, {obstacle_xy[1]:.2f}) in {self.fixed_frame}')
        self.marker_pub.publish(MarkerArray(markers=[ball, line, label]))

    # ---------- robot position via TF ----------
    def update_trail(self):
        try:
            # Time() = "the latest transform you have".
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, self.base_frame, Time())
        except TransformException:
            return
        p = PoseStamped()
        p.header.frame_id = self.fixed_frame
        p.header.stamp = tf.header.stamp
        p.pose.position.x, p.pose.position.y = tf.transform.translation.x, tf.transform.translation.y
        p.pose.orientation = tf.transform.rotation
        last = self.trail.poses[-1].pose.position if self.trail.poses else None
        if last is None or math.hypot(p.pose.position.x - last.x, p.pose.position.y - last.y) > 0.05:
            self.trail.poses.append(p)
            self.trail.poses = self.trail.poses[-2000:]
        self.trail.header.stamp = tf.header.stamp
        self.trail_pub.publish(self.trail)

    def print_tf_tree_once(self):
        if self.tree_printed:
            return
        frames = self.tf_buffer.all_frames_as_yaml()
        if 'base_scan' not in frames:
            return
        self.tree_printed = True
        parents = {}
        current = None
        for line in frames.splitlines():
            if line.strip() and not line.startswith(' '):
                current = line.strip().rstrip(':').strip()
            elif 'parent:' in line and current:
                parents[current] = line.split(':', 1)[1].strip().strip("'\"")
        chain, f = [], 'base_scan'
        while f in parents and len(chain) < 10:
            chain.append(f)
            f = parents[f]
        chain.append(f)
        self.get_logger().info('TF chain for the laser: ' + ' -> '.join(reversed(chain)))
        self.get_logger().info(f'All frames: {", ".join(sorted(set(parents) | set(parents.values())))}')


def main():
    rclpy.init()
    node = ScanInspector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
