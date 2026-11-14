# Stage 6: Blocked paths and failed deliveries (recovery logic, timeouts, debugging)

**Goal:** make the robot behave sensibly when the world doesn't match the plan: a blocked
aisle, a box suddenly in the way, a station that can't be reached.

## How to run it
**In the app** (Navigation mode):
| Button | What it does in Gazebo | What should happen |
|---|---|---|
| **Block aisle** | 3 m wall across the centre aisle | Deliveries to C **detour** around it |
| **Block station A** | 1.2 m box on top of station A | Deliver A **fails safely**: retries, gives up, returns HOME |
| **Box ahead** | 0.5 m box 1.2 m in front of the robot | Local controller steers around it |
| **Clear boxes** | removes all boxes | back to normal |
| **Reset robot to HOME** | removes boxes, teleports the robot HOME, re-initialises AMCL, clears costmaps | a clean, known start (use it after a failure) |
Orange squares on the app map show the boxes. The events line explains every decision.

**Terminal:** `ros2 run warehouse_runner obstacles add aisle` / `add station_A` / `add ahead` / `clear` / `reset`

Files: `scripts/obstacles.py` (spawns boxes with Gazebo's `/spawn_entity` service),
recovery logic inside `scripts/delivery_controller.py`.

---

## Theory 1: two layers of recovery
| Layer | Who | Handles | How |
|---|---|---|---|
| 1. Nav2 behavior tree | Nav2 | small trouble: temporary blockage, stale costmap | replan every 1 s, clear costmaps, spin, wait, back up (≤ 6 times) |
| 2. Mission logic | **our controller** | trouble Nav2 can't fix: unreachable goal, endless circling, long blockage | timeout, stuck detection, retry, skip, return HOME |

Nav2 knows how to drive. It doesn't know about *deliveries*: whether to skip a station or go
home is a **mission** decision, so it belongs in our node.

## Theory 2: the recovery rules in `delivery_controller.py`
| Rule | Parameter (default) | Why |
|---|---|---|
| Leg timeout | `leg_timeout` 150 s | Never wait forever. A normal leg takes ~50 s. |
| Stuck detection | `stuck_timeout` 25 s | If `distance_remaining` (action feedback) hasn't improved by 10 cm in 25 s, the robot is circling or pinned. Cancel. |
| Retry | `max_retries` 2 | Many failures are temporary. Before each retry: **clear both costmaps** (removes "ghost" obstacles), wait 2 s. |
| Give up | `on_failure` = `skip` | After the retries, mark the station failed and continue to the next one. `abort` goes straight HOME. |
| Goal blocked | `goal_check_distance` 2.5 m, `goal_clear_radius` 0.65 m, `goal_blocked_points` 15, `goal_blocked_scans` 3 | The laser shows something solid **on** the goal, so skip it at once, with no retries (retrying beside the obstacle made AMCL get lost). |
| Always end at HOME | – | If even HOME fails, the outcome is `failed` and the robot needs a human. |

Outcomes: `success` (all legs), `partial` (some station failed, robot back HOME), `failed`
(couldn't get HOME / goal), `cancelled`.

## Theory 3: why obstacles are hard
The boxes are **not in the map**. Nav2 only knows about them from the **laser**, through the
costmap obstacle layer:
- **Blocked aisle:** the first plan goes straight through the aisle. When the laser sees the
  wall, the costmap marks it, the 1 s replanning finds the detour, and the robot turns around.
  That costs time and distance, but it's not a failure.
- **Blocked station:** the goal itself is inside an obstacle. The planner can't find a path
  (its 0.5 m tolerance isn't enough for a 1.2 m box), Nav2 recovers 6×, then **aborts**.
  This is where our retries and give-up rule matter.
- **Laser blind spots:** the LDS sees 12 cm–3.5 m in a flat plane. Anything lower or higher
  than the laser is invisible. That's a real limitation of this robot.

## Theory 4: debugging toolkit
| Question | Tool |
|---|---|
| What did the mission decide and why? | `/delivery/events`, mission JSON (`reason` per leg) |
| Is Nav2 even running? | `ros2 lifecycle get /bt_navigator` → `active` |
| What does Nav2 think is blocked? | RViz → global/local costmap displays |
| Where does it want to go? | `/plan` (dashed line in the app) |
| Is localization lost? | yellow AMCL ring far from the white robot dot |
| Replay a failure later | Stage 7 `--bag` records a rosbag per trial |

## Results (7 Nov 2026, each scenario starts from `obstacles reset`)
| Scenario | Outcome | Time | Distance | What happened |
|---|---|---|---|---|
| Normal C (reference) | success | 84.7 s | 14.6 m | – |
| Box (0.5 m) in the centre aisle | success | 86.7 s | 15.5 m | local controller steered around it: +2 s, +0.9 m |
| Centre crossing blocked | success | 164.6 s | 28.7 m | saw the wall, re-planned via the outer corridor: **~2× time and distance** |
| Box on station A, run 1 | **partial** | 210.5 s | 16.6 m | A: 3 attempts, gave up (`stuck`), returned HOME ✔ |
| Box on station A, run 2 | **partial** | 186.5 s | 15.8 m | same behaviour ✔ |

Ground truth (Gazebo) for the blocked-A runs: the robot really stopped **1.24–1.26 m** from A
(beside the box). AMCL was off by **0.14–0.22 m** after seeing the unmapped box, so "HOME" was
reached **~0.3 m** from the true HOME. The goal tolerance is 0.25 m *in AMCL's belief*.

## What went wrong on the way (the real Stage 6 lessons)
1. **The narrow-gap trap.** The first "blocked aisle" box was a thin 0.4 m wall. It left two 0.6 m
   gaps beside it. With a 0.55 m inflation zone, those gaps look *barely* passable to the planner,
   so Nav2 kept choosing them and the controller kept failing inside them. 10–44 recoveries, 333 s,
   then "stuck" on the way home. **Fix:** a blocker that fills the whole crossing (3.0 × 1.6 m).
   The planner then takes the wide corridor straight away. *Planners love the shortest path, even
   when it's a bad one. Inflation and gap width decide whether a route is a trap.*
2. **Lost localization chains failures.** After long struggles, AMCL's estimate drifted. The
   log showed *"None of the points of the global plan were in the local costmap"*, a sign the
   robot is not where it thinks. The next scenario then started from a broken state. **Fix:**
   `obstacles reset` teleports the robot HOME (Gazebo `/set_entity_state`), re-initialises AMCL
   (`/initialpose`, like RViz's "2D Pose Estimate") and clears the costmaps.
3. **A "success" that wasn't.** In one run before the reset fix, Nav2 reported A as *reached*
   while a 1.2 m box sat on it, so localization had been fooled. Nav2 only knows its *belief*.
   **Fix:** the controller now records **simulation ground truth** (true distance to goal, AMCL
   error) in every report. It never uses it for decisions, since a real robot doesn't have it,
   but Stage 7 uses it to count false successes.
4. **Nav2 sometimes never finishes starting.** About 1 start in 6, the lifecycle manager waited
   forever at "Configuring planner_server". The log says *"failed to send response to
   /planner_server/change_state (timeout)"*: a DDS reply got lost while the PC was busy starting
   Gazebo. **Fix:** a startup **watchdog**. The app restarts the simulator if Nav2 isn't active
   after 2 minutes (up to 2 times).
5. **Lost replies from Gazebo services.** A box spawned, but the reply got lost, so our record
   didn't know about it, and the next spawn clashed with its name. **Fix:** unique names, and
   "clear" asks Gazebo itself (`/model_states`) which boxes exist. *Treat a missing reply as
   "unknown", then check the real state.*

6. **(Found in Stage 7) Recovery spins beside an unmapped box make AMCL jump.** The rosbags showed
   AMCL's error jumping from 0.1 to 2.6–2.9 m within seconds of Nav2's spin recoveries next to the
   box on station A. **Fix:** the controller's *goal-blocked* rule. Within 2.5 m of the goal, ≥ 15
   laser hits within 0.65 m of the goal for 3 scans means skip it immediately (`goal_blocked`), no
   retries. Result: blocked-station trials went from 1/4 to **7/7** handled correctly, ~75 s
   instead of up to 232 s, 0 recovery spins. Details in [STAGE7](STAGE7.md).
   *Lesson: sometimes the best recovery is not to try. Retrying the impossible can damage the
   robot's state.*

## Checklist
- [ ] I can explain which failures Nav2 handles and which my controller handles
- [ ] I can explain the stuck rule using action feedback
- [ ] I watched a detour (blocked aisle) and a safe failure (blocked station)
- [ ] I can find the reason for a failed leg in the mission report

Next: **Stage 7**, repeatable trials and real numbers.
