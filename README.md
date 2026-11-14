# Warehouse Runner: learning ROS 2 with a simulated delivery robot

A research/learning project with no physical robot. A simulated TurtleBot3 works in a 10 × 10 m
warehouse (Gazebo Classic + ROS 2 Humble, Ubuntu 22.04 in WSL 2). Over 7 stages it learns to
drive, see, map, navigate, deliver, recover from failures, and be measured.

## Start here: the Warehouse Control app
Double-click **`Warehouse Control.vbs`**. Pick a **Mode**, press **▶ Start**. The Gazebo 3D window
opens by itself when the robot is ready.

| Mode | Starts | Use it for |
|---|---|---|
| Manual: Stage 1-2 | simulator only | driving, the square node, laser + TF in RViz |
| Mapping: Stage 3 | simulator + slam_toolbox | building your own map (explore, save, compare) |
| Navigation: Stage 4-7 | simulator + Nav2 + delivery controller (prebuilt map) | click-to-go, deliveries, obstacles, trials |
| Navigation on MY SLAM map | the same, on your Stage 3 map | Stage 4 on your own map |

| Control | Stage | What it does |
|---|---|---|
| Arrows / **W A S D**, arrow buttons | 1 | Drive by hand (hold) |
| **S1: square** | 1 | Runs your first node, `square_driver` |
| **RViz** | 2 / 4 | Manual and Mapping: laser + TF view. Navigation: Nav2 view with costmaps and the goal tool |
| **S3: explore + save map**, **save map now**, **compare maps** | 3 | SLAM mapping and map quality score |
| **Click on the map** | 4 | Nav2 drives to that point |
| **Deliver A / B / C, Tour, Return home** | 5 | Missions run by the delivery controller (action + state machine) |
| **Block aisle / Block station A / Box ahead / Clear boxes** | 6 | Drop obstacles into Gazebo to test recovery |
| **Reset robot to HOME** | 6-7 | Clean start: no boxes, robot teleported HOME, AMCL re-initialised |
| **Run 5 trials** | 7 | Repeatable trials, recorded and analysed |
| **STOP (Space)** | – | Stops driving, cancels missions, the square, the explorer, trials |

The app map shows: grey = `/map`, red dots = laser, white dot = robot (odometry), yellow ring
= AMCL estimate, dashed yellow = Nav2 plan, orange = boxes.
Stations: HOME green (0, −3.8), A orange (−3.7, 2.8), B blue (3.7, 2.8), C pink (0, 3.8).

## Learning plan
| Stage | Build | Learn | Guide |
|---|---|---|---|
| 1 | Drive the robot | Nodes, topics, velocity commands, launch files | [STAGE1](docs/STAGE1.md) |
| 2 | Show laser scans and robot position | Sensor messages, TF frames, RViz | [STAGE2](docs/STAGE2.md) |
| 3 | Explore and build a map | SLAM (slam_toolbox), map quality | [STAGE3](docs/STAGE3.md) |
| 4 | Navigate to a selected point | Nav2, AMCL, costmaps, planning | [STAGE4](docs/STAGE4.md) |
| 5 | Python delivery controller | Actions, feedback, cancellation, state machines | [STAGE5](docs/STAGE5.md) |
| 6 | Handle blocked paths and failures | Recovery logic, timeouts, debugging | [STAGE6](docs/STAGE6.md) |
| 7 | Repeatable delivery trials | Recording data, measuring performance | [STAGE7](docs/STAGE7.md) |

Each guide has the theory, how to run it, **measured results**, and a checklist.

**Headline results:** the SLAM map found 92% of the walls (5.8 cm error). Nav2 drives equally well
on it (within 3%). Normal deliveries succeeded 9/9 at ±1.5% time. A blocked aisle costs 2.3× time
but still succeeds 3/3. An unreachable, unmapped station *was* the weak spot (1/4 handled correctly;
ground truth exposed a lost robot and a false "success"). The rosbags showed the cause: Nav2's spin
recoveries beside the box made AMCL jump 2–3 m. A sensor-only "goal blocked" rule fixed it:
**7/7** correct, ~75 s, zero recoveries (see [STAGE7](docs/STAGE7.md)).
Result files and charts are in [docs/results](docs/results).

## Folder map
```
ROS_2/
├─ Warehouse Control.vbs            ← double-click to open the app
├─ README.md
├─ docs/STAGE1..7.md, docs/results/ ← guides and measured results
└─ warehouse_runner/                ← the ROS 2 package (linked into ~/ros2_ws/src in Ubuntu)
   ├─ control_app.py                the app (Tkinter + rclpy)
   ├─ launch/  stage1 (sim), stage2 (+laser/TF view), stage3 (+SLAM), warehouse (+Nav2 + controller)
   ├─ scripts/ square_driver (S1) · scan_inspector (S2) · explorer, compare_maps (S3)
   │           delivery_controller, deliver (S5-6) · obstacles (S6) · run_trials, analyze_trials (S7)
   ├─ rviz/stage2.rviz              Stage 2 RViz layout
   ├─ worlds/warehouse.world        walls, shelves, stations
   └─ maps/  warehouse.yaml/.pgm (prebuilt map) · nav2.yaml (Nav2) · slam.yaml (SLAM)
```
In Ubuntu, `~/ros2_ws/` also holds: `maps/` (your SLAM maps), `delivery-results/` (mission
reports), `trials/` (Stage 7 runs), `control-app-logs/` (app logs).

## Terminal commands (Ubuntu / WSL)
Open Ubuntu with `wsl -d Ubuntu-22.04`, then:
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws && colcon build --packages-select warehouse_runner   # rebuild after edits
ros2 launch warehouse_runner stage1.launch.py          # S1: sim + square_driver
ros2 launch warehouse_runner stage2.launch.py          # S2: sim + scan_inspector + RViz
ros2 launch warehouse_runner stage3.launch.py explore:=true   # S3: SLAM + auto explore + save
ros2 launch warehouse_runner warehouse.launch.py       # S4-7: Nav2 + delivery controller
ros2 run warehouse_runner deliver A B C                # S5: send a delivery order
ros2 run warehouse_runner obstacles add aisle          # S6: block the aisle
ros2 run warehouse_runner run_trials --repeats 3       # S7: trials
```
If `ros2 topic …` commands crash with `!rclpy.ok()`, run `ros2 daemon stop` and try again.

## Known quirks (and what the app does about them)
- **Nav2 sometimes never finishes starting** (a lost DDS reply; about 1 start in 6 here). The app
  restarts the simulator automatically if Nav2 isn't active after 2 min.
- **Windows display hiccups** (WSLg) can close Linux windows. If the app closes, just reopen it.
  It reconnects to the simulator that is still running.
- **After a long struggle the robot can be "lost"** (AMCL drift). Press **Reset robot to HOME**.

## Scope and notes
- Simulated deliveries: "unloading" is a 3 s wait. There is no real pickup, battery or arm.
- The prebuilt map matches the world. If you edit the world, rebuild the map in Stage 3.
- Gazebo Classic is end-of-life. It was chosen to match this Humble setup. A future migration should use a supported ROS/Gazebo pairing.
