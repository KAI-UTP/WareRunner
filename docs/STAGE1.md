# Stage 1 — Drive a simulated robot (nodes, topics, velocity commands, launch files)

**Goal:** understand how ROS 2 programs talk to each other, then write your own node that
drives the robot in a square and stops for obstacles.

Files for this stage:
- `warehouse_runner/scripts/square_driver.py` — your node (read it top to bottom; it is commented)
- `warehouse_runner/launch/stage1.launch.py` — starts the simulator (no Nav2) + your node
- **Warehouse Control app**: mode **Manual: Stage 1-2** → **▶ Start**, then drive with the
  arrows / W A S D, or press **S1: square** to run your node.

All commands below run in Ubuntu (WSL). Open a terminal with `wsl -d Ubuntu-22.04`, then in
**every** new terminal run:
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
```
After editing any file in `warehouse_runner`, rebuild:
```bash
cd ~/ros2_ws && colcon build --packages-select warehouse_runner && source install/setup.bash
```

---

## 1. Core ideas (5-minute version)

| Concept | Meaning | In this project |
|---|---|---|
| **Node** | One running program with one job | `/gazebo`, `/robot_state_publisher`, `/square_driver` |
| **Topic** | A named channel that carries one message type | `/cmd_vel`, `/odom`, `/scan` |
| **Publisher / Subscriber** | A node sends to / listens on a topic. Neither knows who is on the other side | `square_driver` publishes `/cmd_vel`; Gazebo's diff-drive plugin subscribes |
| **Message type** | The data structure on a topic | `geometry_msgs/Twist`, `nav_msgs/Odometry`, `sensor_msgs/LaserScan` |
| **QoS** | Delivery rules (reliable vs best-effort, history depth) | Sensors are *best effort*, so subscribers must use `qos_profile_sensor_data` |
| **Parameter** | A setting you can change without editing code | `side_length`, `loops`, `safety_distance` |
| **Launch file** | Python that starts many nodes with settings | `stage1.launch.py` includes `warehouse.launch.py` with `nav:=False` |
| **Sim time** | Nodes use the simulator's `/clock` instead of the wall clock | `use_sim_time: True` |

Data flow in Stage 1:
```
 square_driver ──/cmd_vel (Twist)──▶ Gazebo diff-drive plugin ──moves──▶ simulated robot
       ▲                                                                      │
       ├────────────── /odom (Odometry, ~30 Hz) ◀── wheel odometry ───────────┤
       └────────────── /scan (LaserScan, 5 Hz)  ◀── LiDAR plugin ─────────────┘
```

## 2. Exercise A — explore the running system (no code)

Terminal 1, start the simulator with nobody driving:
```bash
ros2 launch warehouse_runner stage1.launch.py driver:=false
```
Terminal 2, inspect it. Write down what you see:
```bash
ros2 node list                        # who is running?
ros2 topic list -t                    # which channels exist, and what type?
ros2 topic info /cmd_vel -v           # who publishes / subscribes, with QoS
ros2 interface show geometry_msgs/msg/Twist
ros2 topic hz /scan                   # how fast? (expect ~5 Hz)
ros2 topic hz /odom                   # (expect ~30 Hz)
ros2 topic echo /odom --once --field pose.pose
ros2 topic echo /scan --once --field ranges   # 360 numbers, one per degree; index 0 = straight ahead
rqt_graph                             # picture of nodes and topics
```
Drive by hand by publishing raw commands (this is all a "driver" really is):
```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.3}}"
# Ctrl+C, then stop the robot:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```
Or use the keyboard: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

**Questions for your notes:** What happens if you stop publishing — does the robot stop?
Why does `/scan` need best-effort QoS? Which node publishes `/tf`?

## 3. Exercise B — run your node

With the simulator from Exercise A still running:
```bash
ros2 run warehouse_runner square_driver --ros-args -p use_sim_time:=true
ros2 param list /square_driver          # in another terminal while it runs
```
Or do everything in one go: `ros2 launch warehouse_runner stage1.launch.py side_length:=1.5 loops:=2`

How `square_driver.py` works:
1. **Callbacks** (`on_odom`, `on_scan`) only *store* the latest data.
2. A **timer** runs `control_loop` 20 times per second. It is a small **state machine**:
   `WAIT_FOR_ODOM → FORWARD → TURN → FORWARD → … → DONE`.
3. FORWARD: drive until the robot has moved `side_length` from where the side began. Steer
   slightly to hold the side's heading. **Hold still** if the laser sees something closer than
   `safety_distance` within ±30° ahead.
4. TURN: proportional control, so `angular.z = 2 × heading error`, limited to `angular_speed`.
5. It always publishes a zero Twist on exit, so the robot is left stopped.

## 4. Results measured on this setup (6 Nov 2026, headless Gazebo)

| Test | Result |
|---|---|
| Topic rates | `/scan` 5.0 Hz, `/odom` 29.4 Hz |
| 1 m square, v1 (each turn = current heading + 90°) | Corner targets drifted 90°, −178°, −85°, 9°; ended 0.12 m / 8° off |
| 1 m square, v2 (corners = start heading + k·90°, plus heading hold) | Corners exactly 90/180/−90/0°; ended at (0.00, −3.81), −1°; ~49 s |
| Safety stop (6 m side toward east wall) | Stopped and held 0.32–0.34 m from the wall at x = 4.59 m |

The v1 → v2 change is a good first lesson: **small errors add up when every step is measured
relative to the previous one**. Use a fixed reference instead.

Note: these numbers come from `/odom`, which is the robot's own estimate from its wheels.
In simulation it is almost perfect. On a real robot it drifts. That is why Stage 2–3
(laser + map) exist.

## 5. Exercises to make it yours (pick at least three)

1. Change `side_length`, `linear_speed`, `loops` from the command line. Does a faster speed make the corners worse?
2. Revert to the v1 turn (`self.pose[2] + math.pi / 2`) with `loops:=5` and measure how far off it ends.
3. Make it drive a triangle, or any N-sided polygon (`sides` parameter).
4. Instead of holding at an obstacle, turn away and continue (simple obstacle avoidance).
5. Publish your own topic, e.g. `/square_driver/state` (`std_msgs/String`), and `ros2 topic echo` it.
6. Write a tiny second node that subscribes to `/scan` and prints the closest obstacle and its angle.
7. Add a `RegisterEventHandler(OnProcessExit(...))` to the launch file so the simulator shuts down when the driver finishes.

## 6. Stage 1 checklist
- [ ] I can explain node / topic / message / publisher / subscriber in my own words
- [ ] I can find any topic's type, rate and publishers from the command line
- [ ] I moved the robot by publishing `/cmd_vel` directly
- [ ] I ran `square_driver` and changed its parameters
- [ ] I modified the node (one exercise from section 5) and rebuilt it
- [ ] I can read `stage1.launch.py` and say what each line starts

Next: **Stage 2** — visualise `/scan` and the robot's frames (`/tf`) in RViz.
