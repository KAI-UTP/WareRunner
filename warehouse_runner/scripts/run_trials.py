#!/usr/bin/env python3
"""Stage 7: run repeatable delivery trials and record the results.

  ros2 run warehouse_runner run_trials                      # standard plan, 3 repeats
  ros2 run warehouse_runner run_trials --repeats 5 --bag    # also record a rosbag per trial
  ros2 run warehouse_runner run_trials --only A,C_aisle     # a subset of scenarios

Needs warehouse.launch.py (Nav2 + delivery_controller) running.
Every trial: reset (remove boxes, teleport the robot to HOME, re-initialise AMCL, clear costmaps)
-> add the scenario's obstacles -> send the order -> wait for the mission report -> save it. The same conditions every time, so the
results can be compared. Output: ~/ros2_ws/trials/<run>/ trials.csv, legs.csv, missions/*.json,
then analyze_trials writes summary.md + charts.
"""
import argparse
import csv
import json
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.srv import ClearEntireCostmap
from std_srvs.srv import Trigger

# name: (order, obstacles, expected outcome, default repeats weight)
SCENARIOS = {
    'A':          ('A', [], 'success'),
    'B':          ('B', [], 'success'),
    'C':          ('C', [], 'success'),
    'C_aisle':    ('C', ['aisle'], 'success'),     # blocked centre aisle: must detour
    'A_blocked':  ('A', ['station_A'], 'partial'),  # unreachable station: must give up safely and return HOME
}
RESULTS = Path.home() / 'ros2_ws' / 'delivery-results'
BAG_TOPICS = ['/odom', '/scan', '/plan', '/amcl_pose', '/delivery/status', '/delivery/events', '/cmd_vel']


class Runner(Node):
    def __init__(self):
        super().__init__('trial_runner', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.status = {}
        self.create_subscription(String, '/delivery/status', lambda m: self.status.update(json.loads(m.data)), 10)
        self.create_subscription(String, '/delivery/events', lambda m: print('    •', m.data, flush=True), 10)
        self.order_pub = self.create_publisher(String, '/delivery/order', 10)
        self.cancel = self.create_client(Trigger, '/delivery/cancel')
        self.clears = [self.create_client(ClearEntireCostmap, s) for s in
                       ('/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap')]

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_ready(self, timeout=300):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.status.get('nav_ready') and self.status.get('state') == 'IDLE':
                return True
        return False

    def clear_costmaps(self):
        for c in self.clears:
            if c.wait_for_service(timeout_sec=2.0):
                f = c.call_async(ClearEntireCostmap.Request())
                rclpy.spin_until_future_complete(self, f, timeout_sec=3.0)

    def mission(self, order, timeout):
        """Send an order, wait for it to finish, return the mission report dict."""
        old = self.status.get('mission_id')
        self.order_pub.publish(String(data=order))
        end = time.monotonic() + timeout
        started = False
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            mid = self.status.get('mission_id')
            if mid and mid != old:
                started = True
            if started and self.status.get('state') == 'IDLE':
                for _ in range(20):
                    f = RESULTS / f'mission-{mid}.json'
                    if f.exists():
                        return json.loads(f.read_text())
                    self.spin_for(0.2)
                return {'id': mid, 'outcome': 'no_report', 'legs': []}
        # Too long: cancel it, so a hung mission cannot block every following trial.
        if self.cancel.wait_for_service(timeout_sec=2.0):
            f = self.cancel.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        self.wait_ready(120)
        return {'id': self.status.get('mission_id'), 'outcome': 'runner_timeout', 'legs': []}


def obstacles(*args):
    subprocess.run(['ros2', 'run', 'warehouse_runner', 'obstacles', *args],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--only', default='', help='comma list of scenarios: ' + ','.join(SCENARIOS))
    ap.add_argument('--bag', action='store_true', help='record a rosbag per trial')
    ap.add_argument('--timeout', type=float, default=900)
    ap.add_argument('--out', default=str(Path.home() / 'ros2_ws' / 'trials' / time.strftime('%Y%m%d-%H%M%S')))
    args = ap.parse_args()
    names = [n for n in (args.only.split(',') if args.only else SCENARIOS) if n]
    out = Path(args.out).expanduser()
    (out / 'missions').mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Runner()
    print('Waiting for Nav2 and the delivery controller...', flush=True)
    if not node.wait_ready():
        print('Controller not ready. Start warehouse.launch.py first.')
        return 1

    plan = [(n, r + 1) for r in range(args.repeats) for n in names]   # interleave scenarios
    trials, legs = [], []
    t_run = time.monotonic()
    for i, (name, rep) in enumerate(plan, 1):
        order, obs, expected = SCENARIOS[name]
        print(f'\n[{i}/{len(plan)}] scenario {name} (repeat {rep}): order "{order}", obstacles {obs or "none"}', flush=True)
        # Same start every time: no boxes, robot teleported to HOME, AMCL re-initialised, costmaps clean.
        obstacles('reset')
        node.spin_for(3.0)
        if obs:
            obstacles('add', *obs)
            node.spin_for(2.0)
        node.wait_ready()
        bag = None
        if args.bag:
            bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(out / 'bags' / f'{i:02d}_{name}_{rep}'),
                                    *BAG_TOPICS], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            node.spin_for(2.0)
        report = node.mission(order, args.timeout)
        if bag:
            bag.send_signal(signal.SIGINT)
            bag.wait(timeout=20)
        (out / 'missions' / f'{i:02d}_{name}_{rep}.json').write_text(json.dumps(report, indent=2))
        ls = report.get('legs', [])
        row = {'trial': i, 'scenario': name, 'repeat': rep, 'order': order, 'obstacles': '+'.join(obs),
               'expected': expected, 'outcome': report.get('outcome'),
               'as_expected': report.get('outcome') == expected,
               'total_seconds': report.get('total_seconds'), 'total_distance_m': report.get('total_distance_m'),
               'recoveries': sum(l.get('recoveries', 0) for l in ls),
               'retries': sum(max(0, l.get('attempts', 1) - 1) for l in ls),
               'min_clearance_m': min((l['min_clearance_m'] for l in ls if l.get('min_clearance_m')), default=None),
               'failed_legs': ';'.join(f"{l['target']}:{l['reason']}" for l in ls if l.get('success') is False),
               # Simulation ground truth (Gazebo): did "success" really put the robot at the goal?
               'max_true_goal_error_m': max((l['true_goal_error_m'] for l in ls if l.get('success') and 'true_goal_error_m' in l), default=None),
               'max_amcl_error_m': max((l['amcl_error_m'] for l in ls if 'amcl_error_m' in l), default=None),
               'false_successes': sum(1 for l in ls if l.get('success') and l.get('true_goal_error_m', 0) > 0.5),
               'mission_id': report.get('id')}
        trials.append(row)
        for j, l in enumerate(ls, 1):
            legs.append({'trial': i, 'scenario': name, 'leg': j, **{k: l.get(k) for k in
                         ('target', 'success', 'reason', 'attempts', 'seconds', 'distance_m', 'recoveries',
                          'min_clearance_m', 'true_goal_error_m', 'amcl_error_m')}})
        print(f"  -> {row['outcome']} ({'as expected' if row['as_expected'] else 'NOT as expected'}), "
              f"{row['total_seconds']} s, {row['total_distance_m']} m", flush=True)
        for path, rows in (('trials.csv', trials), ('legs.csv', legs)):
            with open(out / path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader(); w.writerows(rows)

    obstacles('reset')
    print(f'\nAll {len(plan)} trials done in {(time.monotonic() - t_run) / 60:.1f} min. Results: {out}', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    subprocess.run(['ros2', 'run', 'warehouse_runner', 'analyze_trials', str(out)])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
