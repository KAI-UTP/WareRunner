#!/usr/bin/env python3
"""Stage 6: drop boxes into the running Gazebo world to create blocked paths.

  ros2 run warehouse_runner obstacles add aisle        # wall across the centre aisle -> forces a detour
  ros2 run warehouse_runner obstacles add station_A    # box on station A -> delivery to A must fail safely
  ros2 run warehouse_runner obstacles add ahead        # box 1.2 m in front of the robot -> local avoidance
  ros2 run warehouse_runner obstacles add 1.0:-2.0     # 0.5 m box at x:y  (or x:y:width:depth)
  ros2 run warehouse_runner obstacles clear            # remove every box added this way
  ros2 run warehouse_runner obstacles list
  ros2 run warehouse_runner obstacles reset            # clear boxes + put the robot back at HOME
                                                       # (Gazebo teleport + AMCL re-initialised + costmaps cleared)

The boxes are NOT in the prebuilt map. Nav2 only learns about them from the laser
(obstacle layer of the costmaps). That is exactly the situation recovery logic is for.
Uses the Gazebo services /spawn_entity and /delete_entity (gazebo_msgs).
"""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry

PRESETS = {  # name: (x, y, width_x, depth_y)
    # Fills the whole centre crossing between the 4 shelves. (A thinner 0.4 m wall left two 0.6 m
    # gaps that Nav2 kept trying to squeeze through - see docs/STAGE6.md.)
    'aisle': (0.0, 0.0, 3.0, 1.6),
    'station_A': (-3.7, 2.8, 1.2, 1.2),
    'station_B': (3.7, 2.8, 1.2, 1.2),
}
STATE = Path.home() / 'ros2_ws' / 'obstacles.json'

SDF = """<?xml version='1.0'?><sdf version='1.6'><model name='{name}'><static>true</static>
<pose>{x} {y} 0.25 0 0 0</pose><link name='link'>
<collision name='c'><geometry><box><size>{w} {d} 0.5</size></box></geometry></collision>
<visual name='v'><geometry><box><size>{w} {d} 0.5</size></box></geometry>
<material><ambient>1 0.45 0.05 1</ambient><diffuse>1 0.45 0.05 1</diffuse></material></visual>
</link></model></sdf>"""


def load():
    try:
        return json.loads(STATE.read_text())
    except (FileNotFoundError, ValueError):
        return {}


def save(boxes):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    STATE.write_text(json.dumps(boxes, indent=1))


def call(node, client, req):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f'{client.srv_name} not available - is Gazebo running?')
    f = client.call_async(req)
    rclpy.spin_until_future_complete(node, f, timeout_sec=15.0)
    return f.result()   # None = the reply got lost (it happens with DDS under load); callers check


def gazebo_obstacles(node):
    """Names of obstacle_* models that really exist in Gazebo (source of truth, not our json)."""
    got = {}
    sub = node.create_subscription(ModelStates, '/model_states', lambda m: got.setdefault('m', m), 10)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)
        if 'm' in got:
            break
    node.destroy_subscription(sub)
    return [n for n in got['m'].name if n.startswith('obstacle_')] if 'm' in got else None


def robot_pose(node):
    got = {}
    sub = node.create_subscription(Odometry, '/odom', lambda m: got.setdefault('m', m), qos_profile_sensor_data)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if 'm' in got:
            break
    node.destroy_subscription(sub)
    if 'm' not in got:
        raise RuntimeError('no /odom - is the simulator running?')
    p = got['m'].pose.pose
    q = p.orientation
    return p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def add(node, what):
    if what in PRESETS:
        x, y, w, d = PRESETS[what]
    elif what == 'ahead':
        rx, ry, ryaw = robot_pose(node)
        x, y, w, d = rx + 1.2 * math.cos(ryaw), ry + 1.2 * math.sin(ryaw), 0.5, 0.5
    else:
        v = [float(t) for t in what.split(':')]
        x, y = v[0], v[1]
        w, d = (v[2], v[3]) if len(v) >= 4 else (0.5, 0.5)
    boxes = load()
    name = f'obstacle_{int(time.time() * 1000) % 100000000}'   # unique, so a lost reply can't cause a name clash
    boxes[name] = {'x': round(x, 2), 'y': round(y, 2), 'w': w, 'd': d, 'preset': what}
    save(boxes)                                                    # record first, so "clear" always knows it
    req = SpawnEntity.Request(name=name, xml=SDF.format(name=name, x=x, y=y, w=w, d=d))
    res = call(node, node.create_client(SpawnEntity, '/spawn_entity'), req)
    if res is None:
        if name not in (gazebo_obstacles(node) or []):
            raise RuntimeError('spawn reply lost and the box is not in Gazebo')
    elif not res.success:
        boxes.pop(name); save(boxes)
        raise RuntimeError(res.status_message)
    print(f'added {name} at ({x:.2f}, {y:.2f}) size {w}x{d} m')


def clear(node):
    names = set(load()) | set(gazebo_obstacles(node) or [])
    client = node.create_client(DeleteEntity, '/delete_entity')
    for name in sorted(names):
        call(node, client, DeleteEntity.Request(name=name))
        print('removed', name)
    left = gazebo_obstacles(node)
    save({})
    if left:
        raise RuntimeError(f'still in Gazebo: {left}')


def reset(node):
    """Known start state for repeatable trials: no boxes, robot at HOME, AMCL told so, clean costmaps."""
    clear(node)
    node.create_publisher(Twist, '/cmd_vel', 10).publish(Twist())
    req = SetEntityState.Request()
    req.state.name = 'warehouse_bot'
    req.state.reference_frame = 'world'
    req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = 0.0, -3.8, 0.01
    req.state.pose.orientation.w = 1.0
    res = call(node, node.create_client(SetEntityState, '/set_entity_state'), req)
    if res is not None and not res.success:
        raise RuntimeError('teleport failed')
    x, y, _ = robot_pose(node)
    if math.hypot(x, y + 3.8) > 0.1:
        raise RuntimeError(f'teleport did not take effect: robot at ({x:.2f}, {y:.2f})')
    # Tell AMCL where the robot really is (same as RViz's "2D Pose Estimate" button).
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x, msg.pose.pose.position.y = 0.0, -3.8
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02
    msg.pose.covariance[35] = 0.01
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
    for srv in ('/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap'):
        c = node.create_client(ClearEntireCostmap, srv)
        if c.wait_for_service(timeout_sec=2.0):
            call(node, c, ClearEntireCostmap.Request())
    print('robot reset to HOME (0, -3.8); AMCL re-initialised; costmaps cleared')


def main():
    args = sys.argv[1:]
    if not args or args[0] not in ('add', 'clear', 'list', 'reset') or (args[0] == 'add' and len(args) < 2):
        print(__doc__)
        return 1
    if args[0] == 'list':
        print(json.dumps(load(), indent=1))
        return 0
    rclpy.init()
    node = Node('obstacles_cli')
    try:
        if args[0] == 'add':
            for what in args[1:]:
                add(node, what)
        elif args[0] == 'reset':
            reset(node)
        else:
            clear(node)
    except RuntimeError as e:
        print('ERROR:', e)
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
