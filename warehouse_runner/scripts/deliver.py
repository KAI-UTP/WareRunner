#!/usr/bin/env python3
"""Send one delivery order to the delivery_controller and follow it until it finishes.

  ros2 run warehouse_runner deliver A          # A, then HOME
  ros2 run warehouse_runner deliver A B C      # tour
  ros2 run warehouse_runner deliver 1.5:-2.0   # any x:y point in the map
Ctrl+C cancels the mission (calls the /delivery/cancel service).
The controller must be running (it starts with warehouse.launch.py).
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    order = ','.join(sys.argv[1:]) or 'A'
    rclpy.init()
    node = Node('deliver_cli')
    status = {}
    node.create_subscription(String, '/delivery/status', lambda m: status.update(json.loads(m.data)), 10)
    node.create_subscription(String, '/delivery/events', lambda m: print('  •', m.data, flush=True), 10)
    pub = node.create_publisher(String, '/delivery/order', 10)
    cancel = node.create_client(Trigger, '/delivery/cancel')
    print('Waiting for the delivery controller and navigation...', flush=True)
    while rclpy.ok() and not status.get('nav_ready'):
        rclpy.spin_once(node, timeout_sec=0.5)
    old_id = status.get('mission_id')
    pub.publish(String(data=order))
    started, shown = False, 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if status.get('mission_id') not in (None, old_id):
                started = True   # a new mission id = our order was accepted (it may already be finished)
            if started and status.get('state') == 'IDLE':
                print(f"Finished: {status.get('last_outcome')}", flush=True)
                break
            if started and time.monotonic() - shown > 3 and status.get('distance_remaining') is not None:
                shown = time.monotonic()
                print(f"    {status.get('state')} -> {status.get('target')}: "
                      f"{status.get('distance_remaining')} m left, recoveries {status.get('recoveries')}", flush=True)
    except KeyboardInterrupt:
        if cancel.wait_for_service(timeout_sec=2):
            f = cancel.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, f, timeout_sec=3)
            print('Cancel requested.')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
