# Stage 5: A Python delivery controller (actions, feedback, cancellation, state machines)

**Goal:** a proper robot "job manager" node that takes delivery orders, drives each leg with
the Nav2 **action**, shows live feedback, can be cancelled, and keeps track of its state.

## How to run it
It starts automatically in Navigation mode (inside `warehouse.launch.py`).
- **App:** Deliver A/B/C, Tour, Return home. The status line shows the controller's state,
  leg, attempt, metres left and recoveries. **STOP** cancels.
- **Terminal:**
  ```bash
  ros2 run warehouse_runner deliver A B C        # send an order and follow it; Ctrl+C cancels
  ros2 topic echo /delivery/status               # live JSON status
  ros2 topic echo /delivery/events               # human-readable log
  ros2 service call /delivery/cancel std_srvs/srv/Trigger
  ```
Files: `scripts/delivery_controller.py` (the node), `scripts/deliver.py` (command-line client).
Reports: Ubuntu `~/ros2_ws/delivery-results/mission-*.json`.

---

## Theory 1: the three ways ROS nodes talk
| | Topic | Service | Action |
|---|---|---|---|
| Pattern | broadcast, many → many | request → reply | goal → feedback… → result |
| Duration | continuous | quick | long (seconds to minutes) |
| Can cancel? | – | no | **yes** |
| Here | `/delivery/order`, `/delivery/status` | `/delivery/cancel` | `navigate_to_pose` |

Driving to a station takes ~50 s, so it **must** be an action. You want progress updates and the
option to stop halfway.

## Theory 2: the life of an action goal
```
send_goal_async ──▶ accepted? ──no──▶ rejected
                       │yes
                       ▼
          feedback (distance_remaining, recoveries, ETA)… many times
                       │
         cancel_goal_async() can interrupt here
                       ▼
          result: SUCCEEDED | ABORTED (Nav2 gave up) | CANCELED
```
Everything is **asynchronous**: you get "futures" and callbacks. The node never blocks while
waiting, so it keeps receiving messages, publishing status and answering the cancel service.

## Theory 3: the state machine
```
 IDLE ──order──▶ SENDING ──accepted──▶ NAVIGATING ──succeeded──▶ UNLOADING (3 s) ──▶ SENDING (next leg)
                                          │ aborted / timeout / stuck                 … last leg ─▶ IDLE
                                          ▼
                                     RECOVERING ──retry──▶ SENDING        (Stage 6)
 any state ──cancel──▶ IDLE (outcome: cancelled)
```
- The node is always in **exactly one** state. Each event (order, accepted, feedback, result,
  timer, cancel) is only allowed to do what makes sense **in that state**.
- Orders while busy are **rejected** with a reason instead of mixing two missions.
- HOME is appended automatically, so a delivery always ends at base. `GOTO x:y` (map clicks) does not return.

## Theory 4: what the report records (per leg)
`success`, `reason`, `attempts`, `seconds`, `distance_m` (from odometry), `recoveries`
(from Nav2's feedback) and `min_clearance_m` (closest laser reading). Stage 7 builds on these.

## Results (7 Nov 2026, headless Gazebo, prebuilt map)
| Test | Outcome | Time | Distance | Notes |
|---|---|---|---|---|
| Deliver C (C → HOME) | success | 84.7 s | 14.57 m | 40.8 s out, 40.9 s back, 1 attempt each, 0 Nav2 recoveries |
| Cancel during A (STOP after 12 s) | cancelled | 12.1 s | 2.08 m | goal cancelled cleanly, report written |
| Return HOME after the cancel | success | 16.1 s | 2.26 m | |
| `GOTO 2.8:-0.2` (no return) | success | 26.2 s | 5.34 m | Stage 4 click goal |

## Two real bugs found while testing (and what they teach)
1. **Goals rejected right after startup.** The controller said "ready" as soon as the
   `navigate_to_pose` action server *existed*. But Nav2 nodes are **lifecycle nodes**:
   unconfigured → inactive → **active**. While `bt_navigator` was still inactive, it rejected
   goals. **Fix:** ask `/bt_navigator/get_state` every 2 s and only count as ready when ACTIVE.
   *Lesson: "the server exists" ≠ "the server is ready".*
2. **False "stuck" on normal trips.** Legs that take ~35 s were cancelled at 25 s as "stuck".
   Logging the raw feedback showed why: the **first feedback for a new goal is still measured on
   the previous goal's path**, which was nearly finished, so ~0.2 m. That became the "best so far",
   and the real 5 m could never beat it. **Fix:** when `distance_remaining` jumps up by > 0.5 m
   (a new or re-planned path), restart progress tracking.
   *Lesson: never trust the first sample of a stream; log the raw data before blaming the robot.*

**Edge cases fixed in the final review** (each verified live):
- *Cancel in the first few milliseconds* (while the goal is still being sent) used to be able to finish
  the mission twice. Now the cancel waits for the goal to be accepted, then cancels it. Tested:
  cancel 50 ms after the order → exactly one "cancelled" report.
- *Two missions in the same second* got the same ID, so the reports overwrote each other and the trial
  runner could wait forever. IDs now include milliseconds. Tested: two instant missions →
  `…37-677` and `…37-742`.
- A failed lifecycle query could have crashed the controller. It's now caught and treated as "not ready".

## Checklist
- [ ] I can say when to use a topic, a service or an action
- [ ] I can describe the goal → feedback → result lifecycle and the three result states
- [ ] I can draw the controller's state machine
- [ ] I cancelled a running mission and saw the outcome in the report

Next: **Stage 6**, what happens when paths are blocked and deliveries fail.
