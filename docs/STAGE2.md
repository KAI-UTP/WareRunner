# Stage 2: See what the robot sees (sensor messages, coordinate frames, RViz)

**Goal:** understand the laser data, the robot's coordinate frames (TF), and how RViz turns
both into a picture.

## How to run it
**In the app** (easiest):
1. Mode **Manual: Stage 1-2** → **▶ Start**. The Gazebo 3D window opens by itself.
2. Press **RViz**. RViz opens with the Stage 2 view and the `scan_inspector` node starts.
3. Drive with the arrow keys / W A S D and watch:
   - **App map:** red dots = live laser hits; bottom line = position and front distance.
   - **RViz:** red squares = `/scan`; coloured axes = TF frames; red ball = closest obstacle,
     with a label; green line = the path you drove.

**In a terminal:** `ros2 launch warehouse_runner stage2.launch.py`

Files: `scripts/scan_inspector.py` (the node), `rviz/stage2.rviz` (the RViz layout),
`launch/stage2.launch.py`.

---

## Theory 1: the laser scan (`sensor_msgs/LaserScan`)
The LiDAR spins and fires a beam every 1°. Each beam returns **one number: the distance** to
the first thing it hits. A `LaserScan` message is just that list of numbers, plus the
information you need to know which direction each one points:

| Field | Value on this robot | Meaning |
|---|---|---|
| `header.frame_id` | `base_scan` | Which frame the numbers are measured in (the laser itself) |
| `header.stamp` | sim time | **When** the scan was taken |
| `angle_min` / `angle_max` | 0° to 360° | Direction of the first and last beam |
| `angle_increment` | 1° | Angle between beams, which gives 360 beams |
| `range_min` / `range_max` | 0.12 to 3.5 m | Readings outside this are invalid |
| `ranges[]` | 360 floats | The distances. `inf` = nothing within 3.5 m |

Beam `i` points at angle `angle_min + i × angle_increment`. Index 0 is straight ahead, 90 is
left, 180 is behind and 270 is right. Angles are counter-clockwise, which is the ROS convention.

**Polar to x, y:** a beam with distance `r` at angle `θ` hits the point
`x = r·cos θ` (forward), `y = r·sin θ` (left). That point is **in the laser's own frame**.

Rates measured here: `/scan` 5 Hz, `/odom` ~30 Hz, `/tf` ~49 Hz.

## Theory 2: coordinate frames and TF
"The wall is 1.2 m away" only means something if you say **from where**. ROS gives every
part of the robot its own coordinate frame, and **TF** keeps track of how they connect.
The measured tree on this robot:

```
odom                     ← fixed to the world (here it matches warehouse coordinates)
 └─ base_footprint       ← robot centre on the floor   (moves: published by Gazebo diff-drive)
     └─ base_link        ← robot body, 1 cm up          (fixed: from the URDF)
         ├─ base_scan    ← the laser, 3.2 cm back, ~17 cm up
         ├─ imu_link
         ├─ wheel_left_link / wheel_right_link
         └─ caster_back_link
```
- **Moving links** (`odom → base_footprint`) change as the robot drives. They are published on `/tf`.
- **Fixed links** (body → laser) never change. They come from the robot description (URDF) via
  `robot_state_publisher` and are published once on `/tf_static`.
- With Nav2 running, one more frame sits on top: `map → odom`. AMCL publishes it to correct
  odometry drift (Stage 4).

Conventions: x forward, y left, z up. Angles are counter-clockwise. Units are metres and radians (REP 103/105).

### What a transform does
A transform between two frames is a **position + rotation**. To move a point from the laser
frame into the odom frame: **rotate by the laser's heading, then shift by the laser's position**.
```
x_odom = tx + cos(yaw)·x_laser − sin(yaw)·y_laser
y_odom = ty + sin(yaw)·x_laser + cos(yaw)·y_laser
```
That is all `transform_point()` in `scan_inspector.py` does. The TF library chains the
links for you: `odom→base_footprint` + `base_footprint→base_link` + `base_link→base_scan`.

### Time matters
The robot moves while data flies around, so TF stores the **history** of every transform.
`scan_inspector` asks "where was the laser **at the moment this scan was taken**?" by passing the
scan's `header.stamp`. If the scan is newer than the last TF message, it falls back to the latest.
Using the wrong time is a classic bug: the laser points look smeared when the robot turns.

## Theory 3: RViz
RViz **simulates nothing**. It's a viewer that subscribes to topics and draws them. Two key ideas:
- **Fixed Frame** (top of the Displays panel): the frame everything is drawn in. Ours is `odom`.
  RViz uses TF to move every message from its own frame into this one.
- **Displays**: one per topic (LaserScan, TF, RobotModel, Path, MarkerArray, Map). Each must
  use QoS that matches its publisher. LaserScan needs **Best Effort**, or nothing appears.

**Markers** let any node draw shapes in RViz: balls, lines, text. `scan_inspector` publishes three
on `/stage2/markers`. They're the standard way to visualise what your code is "thinking".

## Results measured on this setup (6 Nov 2026)
| Check | Result |
|---|---|
| LaserScan format | `base_scan` frame, 360 beams, 0–360°, 1° step, 0.12–3.50 m |
| TF chain found by the node | `odom → base_footprint → base_link → base_scan` |
| Laser position from `tf2_echo odom base_scan` | ~3.2 cm behind robot centre, 0.19 m above the floor |
| Sector distances at HOME | front 1.58, left 2.02, back 1.58, right 1.09 m |
| **TF proof** | Closest hit 1.24 m at −44° in the laser frame → transformed to **(1.47, −2.60)** in odom. The shelf corner is at **(1.45, −2.60)**. ✅ |
| Trail | 11 poses after a 5 s drive, (0.01, −3.80) → (0.53, −3.38) |

## Commands to try (terminal)
```bash
ros2 topic echo /scan --once --field ranges        # the raw 360 numbers
ros2 topic echo /stage2/sectors                     # nearest wall per direction
ros2 run tf2_ros tf2_echo odom base_scan            # live transform between two frames
ros2 run tf2_tools view_frames                      # writes frames_*.pdf: the TF tree diagram
ros2 topic echo /tf_static --once                   # the fixed links from the URDF
```
If `ros2 topic` commands crash with `!rclpy.ok()`, restart the helper with `ros2 daemon stop`, then try again.

## Things to notice (5 minutes in RViz)
1. Change **Fixed Frame** to `base_footprint` and drive. Now the robot stays still and the world moves. Same data, different frame.
2. Turn on **Odometry (/odom)**. Each arrow is a position estimate.
3. Drive next to a shelf. The red ball jumps to whatever is closest. Read its label: laser-frame numbers on top, odom-frame numbers below.
4. Untick **LaserScan** and change its Reliability to *Reliable*, then tick it again. It goes blank: a QoS mismatch.

## Stage 2 checklist
- [ ] I can explain what one number in `ranges[]` means and which direction it points
- [ ] I can explain what a frame is, and why the laser has its own
- [ ] I can draw the TF tree from `odom` down to `base_scan`
- [ ] I understand "rotate, then shift" for moving a point between frames
- [ ] I know RViz only displays topics, and what Fixed Frame does

Next: **Stage 3**, building the map yourself with SLAM instead of using the prebuilt one.
