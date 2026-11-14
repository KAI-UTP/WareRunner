#!/usr/bin/env python3
"""Warehouse Control: one window for all 7 learning stages. GUI and ROS share one loop.

Modes (pick one, press Start):
  Manual (Stage 1-2)      simulator only: drive, square_driver, laser + TF view
  Mapping (Stage 3)       simulator + slam_toolbox: explore, save and compare the map
  Navigation (Stage 4-7)  simulator + Nav2 + delivery_controller on the prebuilt map
  Navigation on my map    the same, but on the map you built in Stage 3
"""
import fcntl, json, math, os, signal, subprocess, time, threading
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMsg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

STATIONS = {'HOME': (0., -3.8), 'A': (-3.7, 2.8), 'B': (3.7, 2.8), 'C': (0., 3.8)}
COLORS = {'HOME': '#34d399', 'A': '#fb923c', 'B': '#60a5fa', 'C': '#e879f9'}
WS = Path.home() / 'ros2_ws'
LOG = WS / 'control-app-logs'
LOG.mkdir(parents=True, exist_ok=True)
SLAM_MAP = WS / 'maps' / 'slam_map.yaml'
MODES = {  # label: (key, launch command)
    'Manual: Stage 1-2 (no Nav2)': ('manual', ['stage1.launch.py', 'gui:=false', 'driver:=false']),
    'Mapping: Stage 3 (SLAM)': ('slam', ['stage3.launch.py', 'gui:=false', 'rviz:=false', 'explore:=false']),
    'Navigation: Stage 4-7 (prebuilt map)': ('nav', ['warehouse.launch.py', 'gui:=false']),
    'Navigation on MY SLAM map (Stage 4)': ('nav_slam', ['warehouse.launch.py', 'gui:=false', f'map:={SLAM_MAP}']),
}
BUSY_STATES = ('SENDING', 'NAVIGATING', 'UNLOADING', 'RECOVERING')


def find_warehouse_launch():
    """(pid, cmdline) of a running 'ros2 launch warehouse_runner ...', else (None, None)."""
    for d in Path('/proc').iterdir():
        if not d.name.isdigit() or int(d.name) == os.getpid():
            continue
        try:
            a = (d / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if b'launch' in a and b'warehouse_runner' in a:
            return int(d.name), b' '.join(a).decode(errors='ignore')
    return None, None


def mode_of(cmdline):
    if 'stage3' in cmdline:
        return 'slam'
    if 'warehouse.launch' in cmdline and 'nav:=false' not in cmdline.lower():
        return 'nav_slam' if 'map:=' in cmdline else 'nav'
    return 'manual'


class ExistingProcess:
    """A simulator this app did not start. Stopped like Ctrl+C in its own terminal."""
    def __init__(self, pid):
        self.pid = pid
        try:
            self.pgid = os.getpgid(pid)
        except ProcessLookupError:
            self.pgid = None
        if self.pgid == os.getpgrp():
            self.pgid = None  # never signal our own group

    def poll(self):
        try:
            stat = Path('/proc', str(self.pid), 'stat').read_text()
            return 0 if stat.split(')')[1].strip().startswith('Z') else None
        except FileNotFoundError:
            return 0

    def wait(self, timeout=12):
        end = time.monotonic() + timeout
        while self.poll() is None:
            if time.monotonic() > end:
                raise subprocess.TimeoutExpired('existing warehouse', timeout)
            time.sleep(.1)
        return 0


def ros_run(name, args, log):
    with (LOG / log).open('w') as f:
        return subprocess.Popen(['ros2', 'run', 'warehouse_runner', name, *args], stdout=f,
                                stderr=subprocess.STDOUT, start_new_session=True)


class App:
    def __init__(self, root):
        self.root = root
        self.proc = self.viewer = self.rviz = self.square = self.inspector = self.explorer = self.trials = None
        self.mode = None
        self.started_at = time.monotonic()
        self.stopping = self.closing = False
        self.odom_at = self.scan_at = self.status_at = 0.
        self.odom_xy = self.amcl_xy = self.scan = None
        self.xy = (0., -3.8, 0.)
        self.driving = (0., 0.); self.held_at = 0.
        self.keys, self.key_timers = set(), {}
        self.ctrl = {}               # last /delivery/status from the controller
        self.plan = []               # last planned path (Stage 4)
        self.click_goal = None
        self.grid = None; self.grid_img = None; self.grid_key = None
        self.obstacles = {}; self.obstacles_read = 0.
        self.auto_viewed = False
        self.drawn_at = 0.
        self.restarts = 0; self.restart_pending = False
        self.bg_results = []         # (callback, returncode) from background() workers

        n = self.node = rclpy.create_node('warehouse_control')
        self.cmd_pub = n.create_publisher(Twist, '/cmd_vel', 10)
        self.order_pub = n.create_publisher(String, '/delivery/order', 10)
        self.cancel_cli = n.create_client(Trigger, '/delivery/cancel')
        n.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        n.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        n.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        n.create_subscription(PathMsg, '/plan', self.on_plan, 10)
        n.create_subscription(String, '/delivery/status', self.on_status, 10)
        n.create_subscription(String, '/delivery/events', lambda m: self.events.set(m.data), 10)
        n.create_subscription(String, '/explorer/status', lambda m: self.events.set('Explorer: ' + m.data), 10)
        self.build_ui()
        self.tick()

    # =============================== UI ===============================
    def build_ui(self):
        root = self.root
        root.title('Warehouse Control'); root.geometry('1100x740'); root.minsize(960, 660)
        root.configure(bg='#101827')
        st = ttk.Style(); st.theme_use('clam')
        st.configure('TFrame', background='#101827')
        st.configure('TLabel', background='#101827', foreground='#e5edf7', font=('Arial', 11))
        st.configure('H.TLabel', font=('Arial', 10, 'bold'), foreground='#98abc1')
        st.configure('TButton', font=('Arial', 10), padding=5)
        main = ttk.Frame(root, padding=(14, 8)); main.pack(fill='both', expand=True)

        top = ttk.Frame(main); top.pack(fill='x')
        ttk.Label(top, text='WAREHOUSE CONTROL', font=('Arial', 16, 'bold')).pack(side='left')
        ttk.Label(top, text='   Mode:').pack(side='left')
        self.mode_var = tk.StringVar(value=list(MODES)[2])
        self.mode_box = ttk.Combobox(top, textvariable=self.mode_var, values=list(MODES), state='readonly', width=36)
        self.mode_box.pack(side='left', padx=6)
        self.start_btn = ttk.Button(top, text='▶ Start', command=self.start); self.start_btn.pack(side='left')
        self.stop_btn = ttk.Button(top, text='■ Shut down', command=self.shutdown); self.stop_btn.pack(side='left', padx=6)

        tools = ttk.Frame(main); tools.pack(fill='x', pady=(6, 0))
        def tbtn(text, cmd):
            b = ttk.Button(tools, text=text, command=cmd); b.pack(side='left', padx=(0, 5)); return b
        self.view_btn = tbtn('Gazebo 3D', self.view)
        self.rviz_btn = tbtn('RViz', self.open_rviz)
        self.square_btn = tbtn('S1: square', self.run_square)
        self.explore_btn = tbtn('S3: explore + save map', self.run_explorer)
        self.save_btn = tbtn('S3: save map now', self.save_map)
        self.compare_btn = tbtn('S3: compare maps', self.compare_maps)

        self.status = tk.StringVar(value='Pick a mode and press Start.')
        ttk.Label(main, textvariable=self.status, font=('Arial', 11, 'bold')).pack(anchor='w', pady=(6, 0))
        self.events = tk.StringVar(value='The Gazebo 3D window opens by itself when the robot is ready.')
        self.events.trace_add('write', lambda *a: print(self.events.get(), flush=True))
        ttk.Label(main, textvariable=self.events, wraplength=1040, font=('Arial', 10)).pack(anchor='w', pady=(2, 6))

        body = ttk.Frame(main); body.pack(fill='both', expand=True)
        self.canvas = tk.Canvas(body, bg='#1b293b', highlightthickness=0, width=460, height=360, cursor='crosshair')
        self.canvas.pack(side='left', fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda e: self.draw())
        self.canvas.bind('<Button-1>', self.on_click)

        panel = ttk.Frame(body, padding=(14, 0, 0, 0)); panel.pack(side='right', fill='y')
        ttk.Label(panel, text='DELIVERY (Stage 5)', style='H.TLabel').pack(anchor='w')
        grid = ttk.Frame(panel); grid.pack(fill='x', pady=(2, 0))
        self.mission_buttons = []
        for i, (label, order) in enumerate([('Deliver A', 'A'), ('Deliver B', 'B'), ('Deliver C', 'C'),
                                            ('Tour A→B→C', 'A,B,C'), ('Return home', 'HOME')]):
            b = ttk.Button(grid, text=label, command=lambda o=order: self.send_order(o))
            b.grid(row=i // 2, column=i % 2, sticky='ew', padx=1, pady=1); self.mission_buttons.append(b)
        grid.columnconfigure(0, weight=1); grid.columnconfigure(1, weight=1)
        ttk.Label(panel, text='Click the map to send the robot there (Stage 4)', font=('Arial', 9)).pack(anchor='w', pady=(2, 6))

        ttk.Label(panel, text='OBSTACLES (Stage 6)', style='H.TLabel').pack(anchor='w')
        og = ttk.Frame(panel); og.pack(fill='x', pady=(2, 6))
        self.obstacle_buttons = []
        for i, (label, arg) in enumerate([('Block aisle', 'aisle'), ('Block station A', 'station_A'),
                                          ('Box ahead', 'ahead'), ('Clear boxes', None),
                                          ('Reset robot to HOME', 'RESET')]):
            b = ttk.Button(og, text=label, command=lambda a=arg: self.obstacle(a))
            b.grid(row=i // 2, column=i % 2, sticky='ew', padx=1, pady=1); self.obstacle_buttons.append(b)
        og.columnconfigure(0, weight=1); og.columnconfigure(1, weight=1)

        ttk.Label(panel, text='TRIALS (Stage 7)', style='H.TLabel').pack(anchor='w')
        self.trials_btn = ttk.Button(panel, text='Run 5 trials (~15 min)', command=self.run_trials)
        self.trials_btn.pack(fill='x', pady=(2, 6))

        ttk.Label(panel, text='MANUAL DRIVE (arrows / WASD)', style='H.TLabel').pack(anchor='w')
        drive = ttk.Frame(panel); drive.pack(pady=(2, 0))
        self.drive_buttons = []
        for text, r, c, v, w in [('↑', 0, 1, .18, 0), ('←', 1, 0, 0, .6), ('↓', 1, 1, -.12, 0), ('→', 1, 2, 0, -.6)]:
            b = ttk.Button(drive, text=text, width=4); b.grid(row=r, column=c, padx=2, pady=1)
            b.bind('<ButtonPress-1>', lambda e, v=v, w=w: self.hold(v, w))
            b.bind('<ButtonRelease-1>', lambda e: self.release())
            b.bind('<Leave>', lambda e: self.release())
            self.drive_buttons.append(b)
        ttk.Button(panel, text='STOP / Cancel everything (Space)', command=self.stop_motion).pack(fill='x', pady=(6, 0))

        root.bind('<FocusOut>', lambda e: self.clear_keys())
        root.bind_all('<space>', lambda e: self.stop_motion())
        root.bind_all('<KeyPress>', self.key_down)
        root.bind_all('<KeyRelease>', self.key_up)
        root.protocol('WM_DELETE_WINDOW', self.close)

    # =============================== ROS inputs ===============================
    @staticmethod
    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_odom(self, msg):
        self.odom_at = time.monotonic(); p = msg.pose.pose
        # Gazebo's odom frame matches the warehouse coordinates, so odom drives the map smoothly.
        self.odom_xy = (p.position.x, p.position.y, self.yaw(p.orientation))

    def on_amcl(self, msg):
        p = msg.pose.pose; self.amcl_xy = (p.position.x, p.position.y, self.yaw(p.orientation))

    def on_scan(self, msg):
        self.scan = (list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        self.scan_at = time.monotonic()

    def on_map(self, msg):
        self.grid = msg; self.grid_key = None   # re-render on next draw

    def on_plan(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses[::3]]

    def on_status(self, msg):
        self.ctrl = json.loads(msg.data); self.status_at = time.monotonic()

    # =============================== state helpers ===============================
    def drive_ready(self):
        return not self.stopping and time.monotonic() - self.odom_at < 3

    def nav_ready(self):
        return self.drive_ready() and time.monotonic() - self.status_at < 3 and self.ctrl.get('nav_ready', False)

    def running(self, p):
        return p is not None and p.poll() is None

    def mission_active(self):
        return time.monotonic() - self.status_at < 3 and self.ctrl.get('state') in BUSY_STATES

    def busy(self):
        return self.mission_active() or self.running(self.square) or self.running(self.explorer) or self.running(self.trials)

    # =============================== keyboard ===============================
    # Holding a key on Linux sends repeated release/press pairs, so a release only counts
    # if no new press arrives within 80 ms.
    KEYS = {'up': (.18, 0), 'w': (.18, 0), 'down': (-.12, 0), 's': (-.12, 0),
            'left': (0, .6), 'a': (0, .6), 'right': (0, -.6), 'd': (0, -.6)}

    def key_down(self, e):
        if isinstance(e.widget, ttk.Combobox):
            return
        k = e.keysym.lower()
        if k not in self.KEYS:
            return
        t = self.key_timers.pop(k, None)
        if t:
            self.root.after_cancel(t)
        self.keys.add(k); self.apply_keys()

    def key_up(self, e):
        k = e.keysym.lower()
        if k in self.keys:
            self.key_timers[k] = self.root.after(80, lambda: self.key_gone(k))

    def key_gone(self, k):
        self.key_timers.pop(k, None); self.keys.discard(k); self.apply_keys()

    def clear_keys(self):
        self.keys.clear(); self.release()

    def apply_keys(self):
        if not self.keys:
            self.release(); return
        v = max([self.KEYS[k][0] for k in self.keys], key=abs)
        w = max([self.KEYS[k][1] for k in self.keys], key=abs)
        self.hold(v, w)

    def publish(self, v=0., w=0.):
        t = Twist(); t.linear.x = float(v); t.angular.z = float(w); self.cmd_pub.publish(t)

    def hold(self, v, w):
        if self.drive_ready() and not self.busy():
            self.driving = (v, w); self.held_at = time.monotonic()

    def release(self):
        if self.driving != (0., 0.):
            self.publish()
        self.driving = (0., 0.)

    # =============================== simulator ===============================
    def attach_existing(self):
        pid, cmd = find_warehouse_launch()
        if pid is None:
            return False
        self.proc = ExistingProcess(pid); self.mode = mode_of(cmd); self.started_at = time.monotonic()
        label = [k for k, v in MODES.items() if v[0] == self.mode][0]
        self.mode_var.set(label)
        self.events.set(f'Connected to a running simulator ({label}). Shut down will stop it.')
        return True

    def start(self):
        if self.proc is not None or self.stopping:
            return
        if self.attach_existing():
            return
        if self.drive_ready():
            self.events.set('A simulator is already publishing /odom. Stop it before starting a new one.'); return
        key, args = MODES[self.mode_var.get()]
        if key == 'nav_slam' and not SLAM_MAP.exists():
            self.events.set('No SLAM map yet. Build one first in "Mapping: Stage 3" (explore + save map).'); return
        self.mode = key
        self.odom_at = self.status_at = 0.; self.ctrl = {}; self.grid = None; self.plan = []
        self.started_at = time.monotonic()
        with (LOG / 'simulator.log').open('w') as f:
            self.proc = subprocess.Popen(['ros2', 'launch', 'warehouse_runner', *args], stdout=f,
                                         stderr=subprocess.STDOUT, start_new_session=True)
        self.status.set(f'Starting {self.mode_var.get()}… allow up to two minutes.')

    def view(self):
        if not self.drive_ready():
            return
        if subprocess.run(['pgrep', '-x', 'gzclient'], capture_output=True).returncode == 0:
            self.events.set('The Gazebo 3D window is already open (check the taskbar).'); return
        env = os.environ.copy(); env['GAZEBO_MODEL_PATH'] = '/opt/ros/humble/share/turtlebot3_gazebo/models'
        with (LOG / 'viewer.log').open('w') as f:
            self.viewer = subprocess.Popen(['gzclient'], env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        self.events.set('Opening the Gazebo 3D window… it can take ~10 s to appear (check the taskbar).')

    def open_rviz(self):
        if not self.drive_ready():
            return
        if self.running(self.rviz):
            self.events.set('RViz is already open (check the taskbar).'); return
        from ament_index_python.packages import get_package_share_directory as share
        if self.mode in ('nav', 'nav_slam'):
            # Stage 4: Nav2's own view: map, costmaps, planned path, and the "Nav2 Goal" tool.
            args = ['-d', share('nav2_bringup') + '/rviz/nav2_default_view.rviz']
            msg = 'RViz (Nav2 view): costmaps, planned path. Use the "Nav2 Goal" tool to send a goal.'
        else:
            # Stage 2-3: laser, TF frames, robot, trail, closest-obstacle markers (+ the growing SLAM map).
            args = ['-d', share('warehouse_runner') + '/rviz/stage2.rviz'] + (['-f', 'map'] if self.mode == 'slam' else [])
            if not self.running(self.inspector):
                self.inspector = ros_run('scan_inspector', ['--ros-args', '-p', 'use_sim_time:=true'], 'scan_inspector.log')
            msg = 'RViz (Stage 2 view): red = /scan, axes = TF frames, red ball = closest obstacle, green = trail.'
        with (LOG / 'rviz.log').open('w') as f:
            self.rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', *args, '--ros-args', '-p', 'use_sim_time:=true'],
                                         stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        self.events.set(msg)

    # =============================== Stage tools ===============================
    def run_square(self):
        if self.drive_ready() and not self.busy() and self.mode == 'manual':
            self.release()
            self.square = ros_run('square_driver', ['--ros-args', '-p', 'use_sim_time:=true'], 'square.log')
            self.events.set('square_driver is running: your Stage 1 node publishes /cmd_vel. STOP ends it.')

    def run_explorer(self):
        if self.drive_ready() and not self.busy() and self.mode == 'slam':
            self.release()
            self.explorer = ros_run('explorer', ['--ros-args', '-p', 'use_sim_time:=true'], 'explorer.log')
            self.events.set('Explorer started: driving the coverage route (~4 min), then saving the map.')

    def background(self, cmd, done, log):
        """Run a short command off the GUI thread; tick() calls done(returncode) afterwards.
        (Tk must only be touched from the main thread, so the worker just queues the result.)"""
        def work():
            try:
                with (LOG / log).open('w') as f:
                    rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=120).returncode
            except Exception:  # noqa: BLE001 - timeout or missing command
                rc = -1
            self.bg_results.append((done, rc))
        threading.Thread(target=work, daemon=True).start()

    def save_map(self):
        if self.mode != 'slam' or not self.drive_ready():
            return
        (WS / 'maps').mkdir(parents=True, exist_ok=True)
        self.events.set('Saving the map…')
        self.background(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(WS / 'maps' / 'slam_map'),
                         '--ros-args', '-p', 'use_sim_time:=true'],
                        lambda rc: self.events.set(f'Map saved: {SLAM_MAP}' if rc == 0 else 'Map save failed (see map_saver.log)'),
                        'map_saver.log')

    def compare_maps(self):
        if not SLAM_MAP.exists():
            self.events.set('No SLAM map saved yet.'); return
        def done(rc):
            try:
                r = json.loads((WS / 'maps' / 'map_comparison.json').read_text())
                self.events.set(f"SLAM map vs real map: wall accuracy {r['wall_accuracy']:.0%}, coverage {r['wall_coverage']:.0%}, "
                                f"mean error {r['mean_error_cm']} cm, free floor found {r['free_coverage']:.0%}. "
                                f"Picture: {WS / 'maps' / 'map_comparison.png'}")
            except Exception:  # noqa: BLE001
                self.events.set('Compare failed (see compare.log).')
        self.events.set('Comparing maps…')
        self.background(['ros2', 'run', 'warehouse_runner', 'compare_maps'], done, 'compare.log')

    def send_order(self, order):
        if not self.nav_ready() or self.busy():
            return
        self.release(); self.click_goal = None
        self.order_pub.publish(String(data=order))

    def on_click(self, e):
        """Stage 4: click anywhere on the map to send a Nav2 goal there (no return HOME)."""
        if not self.nav_ready() or self.busy():
            return
        w, h, s = self.view_geometry()
        x, y = (e.x - w / 2) / s, (h / 2 - e.y) / s
        if not (-4.9 < x < 4.9 and -4.9 < y < 4.9):
            return
        self.click_goal = (x, y); self.release()
        self.order_pub.publish(String(data=f'GOTO {x:.2f}:{y:.2f}'))

    def obstacle(self, what):
        if self.mode not in ('nav', 'nav_slam') or not self.drive_ready():
            return
        if what == 'RESET' and self.mission_active():
            self.events.set('Stop the mission first (STOP), then reset.'); return
        args = ['reset'] if what == 'RESET' else ['add', what] if what else ['clear']
        self.events.set({'RESET': 'Resetting: boxes removed, robot teleported to HOME…', None: 'Removing boxes…'}.get(what, 'Adding box…'))
        self.background(['ros2', 'run', 'warehouse_runner', 'obstacles', *args],
                        lambda rc: (self.events.set(((LOG / 'obstacles.log').read_text().strip().splitlines() or ['done'])[-1]),
                                    setattr(self, 'obstacles_read', 0.)), 'obstacles.log')

    def run_trials(self):
        if not self.nav_ready() or self.busy() or self.mode != 'nav':
            if self.mode == 'nav_slam':
                self.events.set('Trials use the prebuilt map: switch to "Navigation: Stage 4-7".')
            return
        self.release()
        self.trials = ros_run('run_trials', ['--repeats', '1'], 'trials.log')
        self.events.set('Trials running: A, B, C, C with blocked aisle, A blocked. Results: ~/ros2_ws/trials/')

    def stop_motion(self):
        self.release()
        if self.mission_active() and self.cancel_cli.service_is_ready():
            self.cancel_cli.call_async(Trigger.Request())
        for p in (self.square, self.explorer, self.trials):
            if self.running(p):
                try:
                    os.killpg(p.pid, signal.SIGINT)   # their finally-blocks publish a zero Twist
                except ProcessLookupError:
                    pass
        self.publish()
        self.events.set('Stopping…' if self.busy() else 'Stopped.')

    # =============================== shutdown ===============================
    def shutdown(self):
        if self.stopping:
            return
        self.stop_motion(); self.stopping = True; self.status.set('Shutting down…')
        procs = [p for p in [self.trials, self.square, self.explorer, self.inspector, self.rviz, self.viewer, self.proc]
                 if p is not None and p.poll() is None]

        def send(p, sig):
            try:
                if isinstance(p, ExistingProcess):
                    if p.pgid:
                        os.killpg(p.pgid, sig)
                    else:
                        os.kill(p.pid, sig)
                else:
                    os.killpg(p.pid, sig)
            except (ProcessLookupError, PermissionError):
                pass

        def worker():
            for p in procs:
                send(p, signal.SIGINT)
            for p in procs:
                # Ctrl+C first; if ignored (e.g. a background launch), SIGTERM, then SIGKILL.
                for sig, wait in [(signal.SIGTERM, 20), (signal.SIGKILL, 15)]:
                    try:
                        p.wait(timeout=wait); break
                    except subprocess.TimeoutExpired:
                        send(p, sig)
            # A killed launcher can leave its Gazebo server behind; remove it so the next start is clean.
            for d in Path('/proc').iterdir():
                if not d.name.isdigit():
                    continue
                try:
                    a = (d / 'cmdline').read_bytes()
                except (FileNotFoundError, PermissionError, ProcessLookupError):
                    continue
                if a.startswith(b'gzserver\0') and b'warehouse_runner' in a:
                    try:
                        os.kill(int(d.name), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            self.shutdown_done = True
        self.shutdown_done = False
        threading.Thread(target=worker, daemon=True).start()

    def close(self):
        self.closing = True; self.shutdown()

    # =============================== main loop ===============================
    def tick(self):
        for _ in range(15):
            rclpy.spin_once(self.node, timeout_sec=0)
        while self.bg_results:
            done, rc = self.bg_results.pop(0)
            done(rc)
        now = time.monotonic()
        if self.stopping and getattr(self, 'shutdown_done', False):
            self.proc = self.viewer = self.rviz = self.square = self.inspector = self.explorer = self.trials = None
            self.mode = None; self.auto_viewed = False; self.stopping = False
            self.scan = self.amcl_xy = self.grid = None; self.plan = []; self.ctrl = {}; self.click_goal = None
            self.odom_at = self.status_at = 0.
            self.status.set('Warehouse stopped.')
            if self.closing:
                self.node.destroy_node(); rclpy.shutdown(); self.root.destroy(); return
            if self.restart_pending:
                self.restart_pending = False; self.start()
        if self.proc and self.proc.poll() is not None and not self.stopping:
            self.events.set('Simulator exited. Details: ' + str(LOG / 'simulator.log')); self.shutdown()

        drive, ready, busy = self.drive_ready(), self.nav_ready(), self.busy()
        if drive and self.proc is None and not self.stopping:
            self.attach_existing()
        if drive and not self.auto_viewed and not self.stopping:
            self.auto_viewed = True; self.view()
        slow = 'Startup took too long. Shut down and retry. Log: ' + str(LOG / 'simulator.log')
        if self.proc and not drive and not self.stopping and now - self.started_at > 150 and self.events.get() != slow:
            self.events.set(slow)
        # Watchdog: sometimes a DDS reply is lost while Nav2 starts (lifecycle manager waits forever
        # on "Configuring planner_server"). If Nav2 is not active 2 min after OUR start, restart once or twice.
        if (self.mode in ('nav', 'nav_slam') and drive and not ready and not self.stopping
                and isinstance(self.proc, subprocess.Popen) and now - self.started_at > 120):
            if self.restarts < 2:
                self.restarts += 1; self.restart_pending = True
                self.shutdown()
                self.events.set(f'Nav2 did not become active in 2 min (a lost DDS reply during startup). '
                                f'Restarting the simulator automatically ({self.restarts}/2)…')
            elif self.events.get() != slow:
                self.events.set(slow)
        if ready:
            self.restarts = 0

        if not self.stopping and drive:
            c = self.ctrl
            if self.mode in ('nav', 'nav_slam') and not ready:
                self.status.set('● Robot connected · Nav2 starting… (manual driving works now)')
            elif self.mission_active():
                d = c.get('distance_remaining')
                self.status.set(f"● {c['state']} → {c.get('target')}  (leg {c.get('leg')}/{c.get('legs')}, attempt {c.get('attempt')})"
                                + (f" · {d:.1f} m left" if d else '') + f" · Nav2 recoveries {c.get('recoveries', 0)}"
                                + f" · {c.get('elapsed', 0):.0f} s")
            elif self.running(self.trials):
                self.status.set('● Trials running… ' + self.last_line('trials.log', 'scenario'))
            elif self.running(self.explorer):
                self.status.set('● Exploring and mapping (Stage 3)…')
            else:
                hint = {'manual': 'Drive, run the square, or open RViz',
                        'slam': 'Drive or press "explore + save map" to build the map',
                        'nav': 'Choose a delivery or click the map', 'nav_slam': 'Navigating on YOUR map: click or deliver'}
                self.status.set('● Robot ready · ' + hint.get(self.mode, ''))
        if self.square is not None and self.square.poll() is not None:
            self.events.set('Square: ' + self.last_line('square.log', 'square_driver]')); self.square = None
        if self.trials is not None and self.trials.poll() is not None:
            self.events.set('Trials finished. ' + self.last_line('trials.log', 'Results')); self.trials = None
        if self.explorer is not None and self.explorer.poll() is not None:
            self.explorer = None

        is_nav = self.mode in ('nav', 'nav_slam')
        st = lambda ok: 'normal' if ok else 'disabled'
        self.start_btn.configure(state=st(not (self.proc or self.stopping or drive)))
        self.mode_box.configure(state='readonly' if not (self.proc or self.stopping or drive) else 'disabled')
        self.stop_btn.configure(state=st(self.proc and not self.stopping))
        self.view_btn.configure(state=st(drive)); self.rviz_btn.configure(state=st(drive))
        self.square_btn.configure(state=st(drive and not busy and self.mode == 'manual'))
        self.explore_btn.configure(state=st(drive and not busy and self.mode == 'slam'))
        self.save_btn.configure(state=st(drive and self.mode == 'slam'))
        self.compare_btn.configure(state=st(SLAM_MAP.exists()))
        for b in self.drive_buttons:
            b.configure(state=st(drive and not busy))
        for b in self.mission_buttons:
            b.configure(state=st(ready and not busy))
        for b in self.obstacle_buttons:
            b.configure(state=st(drive and is_nav and not self.running(self.trials)))
        self.trials_btn.configure(state=st(ready and not busy and self.mode == 'nav'))

        if self.driving != (0., 0.):
            # A ten-second hold limit also guards lost mouse-release events.
            if drive and not busy and now - self.held_at < 10:
                self.publish(*self.driving)
            else:
                self.release()
        if now - self.obstacles_read > 1.0:
            self.obstacles_read = now
            try:
                self.obstacles = json.loads((WS / 'obstacles.json').read_text()) if is_nav else {}
            except (FileNotFoundError, ValueError):
                self.obstacles = {}
        if now - self.drawn_at > .1:
            if self.odom_xy:
                self.xy = self.odom_xy
            self.drawn_at = now; self.draw()
        self.root.after(50, self.tick)

    def last_line(self, log, needle):
        try:
            lines = [l for l in (LOG / log).read_text(errors='ignore').splitlines() if needle in l]
            return lines[-1].split(']: ', 1)[-1].strip() if lines else ''
        except FileNotFoundError:
            return ''

    # =============================== drawing ===============================
    def view_geometry(self):
        c = self.canvas
        w, h = max(c.winfo_width(), 400), max(c.winfo_height(), 300)
        return w, h, min(w, h) / 11

    def map_image(self, w, h, s):
        """Render /map (OccupancyGrid) into a Tk image matching the canvas scale (cached)."""
        g = self.grid
        if g is None:
            return None
        key = (g.header.stamp.sec, g.header.stamp.nanosec, g.info.width, g.info.height, w, h)
        if key == self.grid_key:
            return self.grid_img
        from PIL import Image, ImageTk
        import numpy as np
        a = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        rgb = np.zeros(a.shape + (3,), dtype=np.uint8); rgb[:] = (27, 41, 59)        # unknown = background
        rgb[(a >= 0) & (a < 50)] = (38, 58, 82)                                          # free floor
        rgb[a >= 50] = (148, 170, 196)                                                   # occupied
        img = Image.fromarray(rgb[::-1])                                                 # row 0 = bottom in ROS
        px = max(1, int(round(g.info.width * g.info.resolution * s)))
        py = max(1, int(round(g.info.height * g.info.resolution * s)))
        self.grid_img = (ImageTk.PhotoImage(img.resize((px, py), Image.NEAREST)),
                         g.info.origin.position.x, g.info.origin.position.y + g.info.height * g.info.resolution)
        self.grid_key = key
        return self.grid_img

    def draw(self):
        c = self.canvas; c.delete('all')
        w, h, s = self.view_geometry()
        def pt(x, y):
            return w / 2 + x * s, h / 2 - y * s
        img = self.map_image(w, h, s)
        if img:
            c.create_image(*pt(img[1], img[2]), image=img[0], anchor='nw')
            c.create_text(12, 32, text='grey = /map (' + ('SLAM, growing' if self.mode == 'slam' else 'Nav2 map') + ')',
                          fill='#98abc1', anchor='w', font=('Arial', 9))
        else:
            c.create_rectangle(*pt(-5, 5), *pt(5, -5), outline='#74869d', width=3)
            for x in [-2, 2]:
                for y in [-1.7, 1.7]:
                    c.create_rectangle(*pt(x - .55, y + .9), *pt(x + .55, y - .9), fill='#345779', outline='#5c7d99')
        for name, b in self.obstacles.items():
            c.create_rectangle(*pt(b['x'] - b['w'] / 2, b['y'] + b['d'] / 2), *pt(b['x'] + b['w'] / 2, b['y'] - b['d'] / 2),
                               fill='#ea580c', outline='#fdba74')
        for n, (x, y) in STATIONS.items():
            px, py = pt(x, y); c.create_oval(px - 10, py - 10, px + 10, py + 10, fill=COLORS[n], outline='')
            c.create_text(px, py + 20, text=n, fill='white', font=('Arial', 10, 'bold'))
        if self.plan and self.mission_active():
            c.create_line(*[v for p in self.plan for v in pt(*p)], fill='#facc15', width=2, dash=(6, 3)) if len(self.plan) > 1 else None
        if self.click_goal:
            gx, gy = pt(*self.click_goal)
            c.create_line(gx - 8, gy - 8, gx + 8, gy + 8, fill='#facc15', width=3)
            c.create_line(gx - 8, gy + 8, gx + 8, gy - 8, fill='#facc15', width=3)
        x, y, a = self.xy; px, py = pt(x, y)
        front = None
        if self.scan and time.monotonic() - self.scan_at < 2:
            # Stage 2: rotate each beam by the robot heading and shift by its position (a TF by hand).
            ranges, amin, inc, rmin, rmax = self.scan
            lx, ly = x - 0.032 * math.cos(a), y - 0.032 * math.sin(a)
            for i in range(0, len(ranges), 2):
                r = ranges[i]
                if not rmin < r < rmax:
                    continue
                b = amin + i * inc
                if abs(math.atan2(math.sin(b), math.cos(b))) < math.radians(15):
                    front = r if front is None else min(front, r)
                qx, qy = pt(lx + r * math.cos(a + b), ly + r * math.sin(a + b))
                c.create_rectangle(qx - 1.5, qy - 1.5, qx + 1.5, qy + 1.5, fill='#f87171', outline='')
        if self.amcl_xy:
            ax, ay = pt(*self.amcl_xy[:2])
            c.create_oval(ax - 13, ay - 13, ax + 13, ay + 13, outline='#fbbf24', width=2, dash=(3, 2))
        c.create_oval(px - 8, py - 8, px + 8, py + 8, fill='white', outline='#101827')
        c.create_line(px, py, px + 19 * math.cos(a), py - 19 * math.sin(a), fill='#34d399', width=3, arrow='last')
        c.create_text(12, 14, text='LIVE WAREHOUSE MAP', fill='#98abc1', anchor='w', font=('Arial', 10))
        c.create_text(w - 10, 14, text='red = laser   yellow ring = AMCL   dashed = Nav2 plan   orange = boxes',
                      fill='#98abc1', anchor='e', font=('Arial', 9))
        info = f'x={x:.2f}  y={y:.2f}  heading={math.degrees(a):.0f}°'
        if front is not None:
            info += f'   ·   laser front {front:.2f} m'
        c.create_text(12, h - 12, text=info, fill='#d1dcea', anchor='w', font=('Arial', 10))


def main():
    lock = (LOG / 'app.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        root = tk.Tk(); root.withdraw(); messagebox.showinfo('Warehouse Control', 'Warehouse Control is already open.'); return
    rclpy.init(); root = tk.Tk(); App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
