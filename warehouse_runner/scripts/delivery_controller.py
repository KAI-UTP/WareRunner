#!/usr/bin/env python3
"""Stage 5 + 6: the delivery controller, a state machine on top of the Nav2 action.

STAGE 5: actions, feedback, cancellation, state machine
  * Orders arrive on /delivery/order (std_msgs/String), e.g. "A", "A,B,C" or "1.5:-2.0" (x:y).
    HOME is added at the end automatically.
  * Each leg sends a NavigateToPose ACTION goal to Nav2. An action is a long task that
    (1) can be accepted or rejected, (2) streams FEEDBACK (distance remaining, recoveries),
    (3) can be CANCELLED, and (4) ends with a RESULT status (succeeded / aborted / canceled).
  * /delivery/cancel (std_srvs/Trigger) cancels the active goal: a SERVICE is a quick request/reply.
  * State machine:
        IDLE -> SENDING -> NAVIGATING -> UNLOADING -> SENDING (next leg) ... -> IDLE
                              |  failure            ^
                              v                     |
                          RECOVERING (clear costmaps, wait) -- retry --+
  * /delivery/status (String, JSON) is published twice a second for the app and the trial runner.

STAGE 6: recovery logic, timeouts, debugging
  * leg_timeout:   give up an attempt after N seconds.
  * stuck_timeout: give up if distance_remaining has not improved by 10 cm for N seconds.
  * max_retries:   retry a failed leg after clearing both costmaps (stale obstacles are a classic cause).
  * on_failure:    'skip' = skip that station and continue; 'abort' = go straight HOME.
  * goal blocked:  within 2.5 m of the goal, if >= 15 laser hits lie within 0.65 m of it for 3 scans,
                   the goal is occupied -> skip it immediately (no retries). Found from the Stage 7
                   rosbags: Nav2's spin recoveries next to an unmapped box made AMCL jump 2-3 m.
  * Every decision is logged on /delivery/events with a reason, and each mission writes a JSON
    report to ~/ros2_ws/delivery-results/ (time, distance, recoveries, retries, clearance per leg).

  ros2 run warehouse_runner delivery_controller --ros-args -p use_sim_time:=true
"""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

STATIONS = {'HOME': (0.0, -3.8), 'A': (-3.7, 2.8), 'B': (3.7, 2.8), 'C': (0.0, 3.8)}
BUSY = ('SENDING', 'NAVIGATING', 'UNLOADING', 'RECOVERING')


class DeliveryController(Node):
    def __init__(self):
        super().__init__('delivery_controller')
        p = self.declare_parameter
        self.unload_s = p('unload_seconds', 3.0).value
        self.leg_timeout = p('leg_timeout', 150.0).value
        self.stuck_timeout = p('stuck_timeout', 25.0).value
        self.max_retries = p('max_retries', 2).value
        self.on_failure = p('on_failure', 'skip').value
        self.results_dir = Path(p('results_dir', str(Path.home() / 'ros2_ws' / 'delivery-results')).value)
        # Goal-blocked check (Stage 6 fix): when close to the goal, if the laser sees a solid object
        # ON the goal, skip it at once - before Nav2's spin recoveries next to it confuse AMCL.
        self.check_dist = p('goal_check_distance', 2.5).value       # only judge from this close (m)
        self.clear_radius = p('goal_clear_radius', 0.65).value      # "on the goal" = within this radius (m)
        self.block_points = p('goal_blocked_points', 15).value      # laser hits needed per scan
        self.block_scans = p('goal_blocked_scans', 3).value         # consecutive scans needed (5 Hz)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_global = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.status_pub = self.create_publisher(String, '/delivery/status', 10)
        self.event_pub = self.create_publisher(String, '/delivery/events', 10)
        self.create_subscription(String, '/delivery/order', self.on_order, 10)
        self.create_service(Trigger, '/delivery/cancel', self.on_cancel)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        # SIMULATION-ONLY ground truth from Gazebo. Never used for decisions (a real robot has no
        # such thing) - only written into the reports, to check whether "success" was real.
        self.truth_xy = None
        self.amcl_xy = None
        self.amcl_yaw = 0.0
        self.amcl_std = None
        self.blocked_streak = 0
        self.create_subscription(ModelStates, '/model_states', self.on_truth, 10)

        self.state = 'IDLE'
        self.mission = None
        self.leg_i = 0
        self.goal = None
        self.cancel_reason = None
        self.feedback = {'distance_remaining': None, 'recoveries': 0, 'eta': None}
        self.best_remaining = math.inf
        self.last_progress = 0.0
        self.attempt_start = 0.0
        self.wait_until = 0.0
        self.localized = False
        self.last_xy = None
        self.min_clearance = math.inf
        self.message = 'Waiting for orders.'
        self.last_outcome = None
        # Nav2 nodes are "lifecycle" nodes: the action server exists before it is ACTIVE, and
        # goals sent too early are rejected. Ask bt_navigator for its state every 2 s.
        self.bt_state = self.create_client(GetState, '/bt_navigator/get_state')
        self.bt_active = False
        self.create_timer(2.0, self.check_bt)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info('Delivery controller ready. Send orders on /delivery/order.')

    # ---------------- inputs ----------------
    def on_amcl(self, msg):
        self.localized = True
        p, q, c = msg.pose.pose.position, msg.pose.pose.orientation, msg.pose.covariance
        self.amcl_xy = (p.x, p.y)
        self.amcl_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.amcl_std = math.sqrt(max(c[0] + c[7], 0.0))   # AMCL's own position uncertainty (m)

    def on_truth(self, msg):
        if 'warehouse_bot' in msg.name:
            p = msg.pose[msg.name.index('warehouse_bot')].position
            self.truth_xy = (p.x, p.y)

    def record_truth(self, leg):
        """Where the robot REALLY ended vs. the goal, and how wrong AMCL was (simulation only)."""
        if self.truth_xy is None:
            return
        leg['true_goal_error_m'] = round(math.dist(self.truth_xy, (leg['x'], leg['y'])), 3)
        if self.amcl_xy is not None:
            leg['amcl_error_m'] = round(math.dist(self.truth_xy, self.amcl_xy), 3)
        if self.amcl_std is not None:
            leg['amcl_std_m'] = round(self.amcl_std, 3)

    def on_odom(self, msg):
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_xy and self.state == 'NAVIGATING':
            self.leg()['distance_m'] += math.dist(self.last_xy, xy)
        self.last_xy = xy

    def on_scan(self, msg):
        if self.state != 'NAVIGATING':
            return
        close = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if close:
            self.min_clearance = min(self.min_clearance, min(close))
        self.check_goal_blocked(msg)

    def check_goal_blocked(self, scan):
        """Is something solid sitting ON the goal? Uses only the robot's own laser + AMCL pose.

        Validated offline on the Stage 7 rosbags: fired in all 3 blocked-station trials (34-37 s,
        before Nav2's first spin recovery at 37-39 s) and never in the 12 other trials.
        """
        if self.amcl_xy is None or self.goal is None or self.cancel_reason is not None:
            return
        leg = self.leg()
        goal = (leg['x'], leg['y'])
        x, y = self.amcl_xy
        if math.dist((x, y), goal) > self.check_dist:
            self.blocked_streak = 0
            return
        hits = 0
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < scan.range_max:
                a = self.amcl_yaw + scan.angle_min + i * scan.angle_increment
                if math.dist((x + r * math.cos(a), y + r * math.sin(a)), goal) < self.clear_radius:
                    hits += 1
        self.blocked_streak = self.blocked_streak + 1 if hits >= self.block_points else 0
        if self.blocked_streak >= self.block_scans:
            self.cancel_reason = 'goal_blocked'
            self.event(f'{leg["target"]} is blocked: {hits} laser hits within {self.clear_radius} m of it. '
                       'Skipping it now (no retries, no recovery spins next to the obstacle).')
            self.goal.cancel_goal_async()

    def parse(self, text):
        # "GOTO x:y" = Stage 4 click-to-navigate: one goal, no return HOME.
        go_only = text.strip().upper().startswith('GOTO')
        if go_only:
            text = text.strip()[4:]
        legs = []
        for tok in [t.strip() for t in text.replace(' ', ',').split(',') if t.strip()]:
            if tok.upper() in STATIONS:
                name, xy = tok.upper(), STATIONS[tok.upper()]
            elif ':' in tok:
                x, y = (float(v) for v in tok.split(':'))
                name, xy = f'({x:.1f},{y:.1f})', (x, y)
            else:
                raise ValueError(f'unknown destination "{tok}"')
            legs.append({'target': name, 'x': xy[0], 'y': xy[1]})
        if go_only and not legs:
            raise ValueError('GOTO needs a point, e.g. "GOTO 1.5:-2.0"')
        if not go_only and (not legs or legs[-1]['target'] != 'HOME'):
            legs.append({'target': 'HOME', 'x': STATIONS['HOME'][0], 'y': STATIONS['HOME'][1]})
        for leg in legs:
            leg.update(success=None, reason=None, attempts=0, seconds=0.0, distance_m=0.0,
                       recoveries=0, min_clearance_m=None, started=None)
        return legs

    def on_order(self, msg):
        if self.state in BUSY:
            self.event(f'Order "{msg.data}" rejected: busy with mission {self.mission["id"]}.')
            return
        if not self.nav_ready():
            self.event(f'Order "{msg.data}" rejected: navigation is not ready yet.')
            return
        try:
            legs = self.parse(msg.data)
        except ValueError as e:
            self.event(f'Order rejected: {e}')
            return
        mission_id = time.strftime('%Y%m%d-%H%M%S') + f'-{int(time.time() * 1000) % 1000:03d}'  # unique even for quick missions
        self.mission = {'id': mission_id, 'order': msg.data, 'legs': legs,
                        'started_wall': time.time(), 'outcome': None,
                        'settings': {'leg_timeout': self.leg_timeout, 'stuck_timeout': self.stuck_timeout,
                                     'max_retries': self.max_retries, 'on_failure': self.on_failure}}
        self.mission_t0 = time.monotonic()
        self.leg_i = 0
        self.event(f'Mission {self.mission["id"]} accepted: ' + ' -> '.join(l['target'] for l in legs))
        self.start_leg()

    def on_cancel(self, _req, resp):
        if self.state not in BUSY:
            resp.success, resp.message = False, 'Nothing to cancel.'
            return resp
        self.cancel_reason = 'user'
        if self.goal is not None:
            self.goal.cancel_goal_async()          # on_result() will finish the mission
        elif self.state in ('UNLOADING', 'RECOVERING'):
            self.finish('cancelled', 'Cancelled by user.')
        # SENDING: the goal is on its way; on_accepted() sees cancel_reason and cancels it.
        resp.success, resp.message = True, 'Cancelling.'
        return resp

    # ---------------- the state machine ----------------
    def leg(self):
        return self.mission['legs'][self.leg_i]

    def start_leg(self):
        leg = self.leg()
        leg['attempts'] += 1
        if leg['started'] is None:
            leg['started'] = time.monotonic()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'   # stamp 0 = "use the latest transform"
        goal.pose.pose.position.x, goal.pose.pose.position.y = leg['x'], leg['y']
        goal.pose.pose.orientation.w = 1.0
        self.feedback = {'distance_remaining': None, 'recoveries': 0, 'eta': None}
        self.best_remaining, self.last_progress = math.inf, time.monotonic()
        self.attempt_start = time.monotonic()
        self.cancel_reason = None
        self.min_clearance = math.inf
        self.blocked_streak = 0
        self.state = 'SENDING'
        self.message = f'Leg {self.leg_i + 1}/{len(self.mission["legs"])}: going to {leg["target"]}' + \
                       (f' (attempt {leg["attempts"]})' if leg['attempts'] > 1 else '')
        self.event(self.message)
        self.nav.send_goal_async(goal, feedback_callback=self.on_feedback).add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        try:
            handle = future.result()
        except Exception as e:  # noqa: BLE001
            self.attempt_failed(f'send_failed: {e}')
            return
        if not handle.accepted:
            if self.cancel_reason == 'user':
                self.finish('cancelled', 'Cancelled by user.')
            else:
                self.attempt_failed('rejected_by_nav2')
            return
        self.goal = handle
        self.state = 'NAVIGATING'
        if self.cancel_reason == 'user':
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, msg):
        fb = msg.feedback
        d = float(fb.distance_remaining)
        self.feedback = {'distance_remaining': round(d, 2), 'recoveries': int(fb.number_of_recoveries),
                         'eta': fb.estimated_time_remaining.sec}
        if d <= 0.0:
            return
        if d < self.best_remaining - 0.10:          # real progress
            self.best_remaining, self.last_progress = d, time.monotonic()
        elif d > self.best_remaining + 0.5:
            # A new or re-planned path (e.g. a detour). Also, Nav2's first feedback for a new goal
            # can still be measured on the PREVIOUS goal's path (tiny distance). Restart tracking.
            self.best_remaining, self.last_progress = d, time.monotonic()

    def on_result(self, future):
        self.goal = None
        status = future.result().status
        leg = self.leg()
        leg['recoveries'] += self.feedback['recoveries']
        if self.min_clearance < math.inf:
            prev = leg['min_clearance_m']
            leg['min_clearance_m'] = round(self.min_clearance if prev is None else min(prev, self.min_clearance), 3)
        self.record_truth(leg)
        if status == GoalStatus.STATUS_SUCCEEDED:
            leg['success'], leg['seconds'] = True, round(time.monotonic() - leg['started'], 1)
            self.event(f'Reached {leg["target"]} in {leg["seconds"]} s.')
            self.next_leg(unload=leg['target'] != 'HOME')
        elif status == GoalStatus.STATUS_CANCELED and self.cancel_reason == 'user':
            self.finish('cancelled', 'Cancelled by user.')
        elif status == GoalStatus.STATUS_CANCELED and self.cancel_reason in ('timeout', 'stuck', 'goal_blocked'):
            self.attempt_failed(self.cancel_reason)
        else:
            self.attempt_failed('aborted_by_nav2' if status == GoalStatus.STATUS_ABORTED else f'status_{status}')

    def attempt_failed(self, reason):
        self.goal = None
        leg = self.leg()
        # A blocked goal won't unblock itself by retrying - and every retry means more recovery
        # spins next to the obstacle, which is exactly what made AMCL lose track (Stage 7 bags).
        if reason != 'goal_blocked' and leg['attempts'] <= self.max_retries:
            self.state = 'RECOVERING'
            self.message = f'{leg["target"]}: attempt {leg["attempts"]} failed ({reason}). Clearing costmaps, retrying.'
            self.event(self.message)
            for client in (self.clear_global, self.clear_local):
                if client.service_is_ready():
                    client.call_async(ClearEntireCostmap.Request())
            self.wait_until = time.monotonic() + 2.0
            return
        leg['success'], leg['reason'] = False, reason
        leg['seconds'] = round(time.monotonic() - leg['started'], 1)
        self.event(f'{leg["target"]} FAILED after {leg["attempts"]} attempts ({reason}).')
        if leg['target'] == 'HOME' and self.leg_i == len(self.mission['legs']) - 1:
            self.finish('failed', 'Could not return HOME. Robot needs help.')
        elif self.on_failure == 'abort':
            for l in self.mission['legs'][self.leg_i + 1:-1]:
                l['success'], l['reason'] = False, 'skipped_after_abort'
            self.leg_i = len(self.mission['legs']) - 2
            self.next_leg(unload=False)
        else:
            self.next_leg(unload=False)

    def next_leg(self, unload):
        if self.cancel_reason == 'user':
            self.finish('cancelled', 'Cancelled by user.')
            return
        if self.leg_i + 1 >= len(self.mission['legs']):
            legs = self.mission['legs']
            if all(l['success'] for l in legs):
                self.finish('success', 'Mission complete.')
            elif legs[-1]['success']:
                self.finish('partial', 'Returned HOME, but some deliveries failed.')
            else:
                self.finish('failed', f'Could not reach {legs[-1]["target"]}.')
            return
        self.leg_i += 1
        if unload:
            self.state = 'UNLOADING'
            self.message = f'Unloading at {self.mission["legs"][self.leg_i - 1]["target"]}...'
            self.wait_until = time.monotonic() + self.unload_s
        else:
            self.start_leg()

    def tick(self):
        now = time.monotonic()
        if self.state in ('UNLOADING', 'RECOVERING') and now >= self.wait_until:
            if self.cancel_reason == 'user':
                self.finish('cancelled', 'Cancelled by user.')
            else:
                self.start_leg()
        elif self.state == 'NAVIGATING' and self.goal is not None and self.cancel_reason is None:
            if now - self.attempt_start > self.leg_timeout:
                self.cancel_reason = 'timeout'
                self.event(f'Timeout after {self.leg_timeout:.0f} s - cancelling goal.')
                self.goal.cancel_goal_async()
            elif now - self.last_progress > self.stuck_timeout:
                self.cancel_reason = 'stuck'
                self.event(f'No progress for {self.stuck_timeout:.0f} s - robot is stuck, cancelling goal.')
                self.goal.cancel_goal_async()

    def finish(self, outcome, message):
        self.state = 'IDLE'
        self.goal = None
        self.message = message
        self.last_outcome = outcome
        m = self.mission
        m['outcome'] = outcome
        m['total_seconds'] = round(time.monotonic() - self.mission_t0, 1)
        m['total_distance_m'] = round(sum(l['distance_m'] for l in m['legs']), 2)
        for l in m['legs']:
            l['distance_m'] = round(l['distance_m'], 2)
            l.pop('started', None)
        self.results_dir.mkdir(parents=True, exist_ok=True)
        path = self.results_dir / f'mission-{m["id"]}.json'
        path.write_text(json.dumps(m, indent=2))
        self.event(f'Mission {m["id"]} {outcome.upper()}: {message} Report: {path}')
        self.publish_status()

    # ---------------- outputs ----------------
    def check_bt(self):
        if self.bt_state.service_is_ready():
            self.bt_state.call_async(GetState.Request()).add_done_callback(self.on_bt_state)
        else:
            self.bt_active = False

    def on_bt_state(self, fut):
        try:
            self.bt_active = fut.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        except Exception:  # noqa: BLE001 - a failed query must never crash the controller
            self.bt_active = False

    def nav_ready(self):
        return self.nav.server_is_ready() and self.localized and self.bt_active

    def event(self, text):
        self.get_logger().info(text)
        self.event_pub.publish(String(data=text))

    def publish_status(self):
        m = self.mission
        busy = self.state in BUSY and m is not None
        s = {'state': self.state, 'nav_ready': self.nav_ready(), 'message': self.message,
             'mission_id': m['id'] if m else None, 'last_outcome': self.last_outcome}
        if busy:
            leg = self.leg()
            s.update(target=leg['target'], leg=self.leg_i + 1, legs=len(m['legs']), attempt=leg['attempts'],
                     elapsed=round(time.monotonic() - self.mission_t0, 1), **self.feedback)
        self.status_pub.publish(String(data=json.dumps(s)))


def main():
    rclpy.init()
    node = DeliveryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
