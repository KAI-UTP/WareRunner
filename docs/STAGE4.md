# Stage 4: Navigate to a selected location (Nav2, path planning, obstacle avoidance)

**Goal:** click anywhere on the map and let Nav2 plan a path and drive there, avoiding
obstacles. First on the prebuilt map, then on **the map you built in Stage 3**.

## How to run it
**In the app:**
1. Mode **Navigation: Stage 4-7 (prebuilt map)** → **Start**. Wait for "Robot ready" (~40 s).
2. **Click anywhere on the map.** A yellow ✕ marks the goal, a dashed yellow line shows Nav2's
   planned path, and the yellow ring is AMCL's position estimate.
3. **RViz** opens Nav2's own view: costmaps, plan, and the **Nav2 Goal** tool (another way to send goals).
4. Mode **Navigation on MY SLAM map** does the same on your Stage 3 map.

**In a terminal:**
```bash
ros2 launch warehouse_runner warehouse.launch.py                                   # prebuilt map
ros2 launch warehouse_runner warehouse.launch.py map:=$HOME/ros2_ws/maps/slam_map.yaml   # your map
ros2 topic pub --once /delivery/order std_msgs/msg/String "{data: 'GOTO 2.8:-0.2'}"
```
Settings: `maps/nav2.yaml`.

---

## Theory 1: what Nav2 is
Nav2 is a team of nodes, each with one job:

| Node | Job |
|---|---|
| `map_server` | Loads the .pgm/.yaml map and publishes `/map` |
| `amcl` | **Localization:** where am I on that map? Publishes `map → odom` |
| `planner_server` | **Global planner:** finds a path from here to the goal over the whole map (`/plan`) |
| `controller_server` | **Local controller:** turns the path into `/cmd_vel` 20× per second, dodging what the laser sees |
| `behavior_server` | **Recovery moves:** spin, back up, wait |
| `bt_navigator` | **The boss:** a Behavior Tree that runs plan → follow → recover, and offers the `navigate_to_pose` **action** |
| `lifecycle_manager` | Starts all of the above in the right order (configure → activate) |

## Theory 2: localization with AMCL (particle filter)
AMCL keeps ~500–2000 **particles**. Each one is a guess "the robot is here, facing this way".
- When the robot moves, every particle moves the same way, plus some random noise.
- When a scan arrives, each particle is scored: "if I were here, would the laser see what it sees?"
- Bad guesses die, good ones are copied (resampling). The cloud shrinks around the true pose.
- The result is the `map → odom` transform (the drift correction) and `/amcl_pose` (the yellow ring in the app).

## Theory 3: costmaps, the robot's "danger map"
Nav2 doesn't plan on the raw map. It uses **costmaps**: grids where each cell costs 0–254.

| Layer | Source |
|---|---|
| static layer | the walls from `/map` |
| obstacle layer | things the **laser** sees now (boxes, people); not in the map |
| inflation layer | a "keep-away" zone around everything. Cost falls off with distance (`inflation_radius 0.55 m`) |

- **Global costmap:** the whole warehouse, used by the planner.
- **Local costmap:** a 3 × 3 m window around the robot, used by the controller and updated 5×/s.

`robot_radius: 0.12 m`. Any cell closer than this to an obstacle is "lethal".

## Theory 4: planning and following
- **Global planner (NavFn, Dijkstra):** spreads out from the goal across the costmap and finds
  the cheapest path. It prefers the middle of aisles because inflation makes cells near walls expensive.
- **Local controller (DWB):** 20× per second it tries ~400 possible (speed, turn) pairs, simulates
  each for 1.7 s, and scores them with **critics**: stay on the path, head to the goal, don't hit
  obstacles, don't oscillate. The best pair is sent as `/cmd_vel`. Max speed is 0.22 m/s.
- **Goal checker:** done when within 0.25 m and 0.25 rad of the goal.

## Theory 5: the Behavior Tree
`bt_navigator` runs an XML behavior tree, roughly:
```
NavigateRecovery
 ├─ PipelineSequence: ComputePathToPose (replans every 1 s) → FollowPath
 └─ Recovery (if the above fails, up to 6×): clear costmaps → spin → wait → back up
```
That's why Nav2 can get itself out of small trouble. Stage 6 adds **your own** recovery
on top, for the trouble Nav2 can't fix.

## Results (7 Nov 2026, headless Gazebo): prebuilt map vs. YOUR SLAM map
| Test | Prebuilt map | Your SLAM map (Stage 3) |
|---|---|---|
| Click goal HOME → (2.8, −0.2) | ✅ 26.2 s, 5.34 m driven | ✅ 27.0 s, 5.43 m |
| Planned path length (first `/plan`) | 5.46 m (straight line 4.56 m) | 5.52 m |
| Back to HOME | ✅ 31.1 s | ✅ 33.4 s |
| Deliver C and return | ✅ 84.7 s, 14.6 m | ✅ 87.4 s, 15.0 m |
| Nav2 recoveries needed | 0 | 0 |
| Nav2 startup to "active" | ~30 s | ~30 s |

**Reading the result:**
- The planned path is ~20% longer than the straight line. Inflation makes the planner keep away
  from the shelf corners and stay in the middle of the aisle. That's the costmap at work.
- **The robot navigates as well on its own map as on the prebuilt one**, within 3% on time and
  distance. So the Stage 3 map (92% wall coverage, 5.8 cm error) is good enough for Nav2. The
  small extra distance comes from slightly thicker walls in the SLAM map, which inflate a bit more.
- With nothing unexpected in the way, Nav2 needed zero recoveries. Stage 6 shows what happens when there is something.

## Checklist
- [ ] I can name what amcl, planner_server, controller_server and bt_navigator each do
- [ ] I can explain the three costmap layers and what inflation is for
- [ ] I know the difference between the global plan and the local controller
- [ ] I navigated on my own SLAM map

Next: **Stage 5**, a Python delivery controller that uses Nav2 through its action interface.
