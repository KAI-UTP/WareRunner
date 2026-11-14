# Delivery trials: fix-blocked-20261108-0027

6 trials. Behaved as expected: **100%**

| Scenario | n | Expected | Outcomes | As expected | Time (s) | Distance (m) | Nav2 recoveries | Retries | Min clearance (m) |
|---|---|---|---|---|---|---|---|---|---|
| A_blocked | 6 | partial | partial 6 | 100% | 74.67 ± 2.58 | 13.49 ± 0.44 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.56 |

**Ground truth check (simulation only, from Gazebo):**

| Scenario | True distance to goal at "success" (m, worst leg) | AMCL error (m, worst leg) | False successes (>0.5 m off) |
|---|---|---|---|
| A_blocked | 0.35 ± 0.05 | 0.2 ± 0.02 | 0 |

Time and distance cover the whole mission (out and back to HOME). Values are mean ± standard deviation.
Min clearance is the closest laser reading during the mission (lowest across trials).
A false success = Nav2 reported the goal reached, but Gazebo shows the robot more than 0.5 m away.
