# Delivery trials: fix-regression-20261108-0027

5 trials. Behaved as expected: **100%**

| Scenario | n | Expected | Outcomes | As expected | Time (s) | Distance (m) | Nav2 recoveries | Retries | Min clearance (m) |
|---|---|---|---|---|---|---|---|---|---|
| A | 1 | success | success 1 | 100% | 96.4 ± 0.0 | 16.82 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.64 |
| B | 1 | success | success 1 | 100% | 98.7 ± 0.0 | 16.74 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.63 |
| C | 1 | success | success 1 | 100% | 89.0 ± 0.0 | 14.84 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 1.07 |
| C_aisle | 1 | success | success 1 | 100% | 195.5 ± 0.0 | 33.4 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.44 |
| A_blocked | 1 | partial | partial 1 | 100% | 76.2 ± 0.0 | 13.89 ± 0.0 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.57 |

**Ground truth check (simulation only, from Gazebo):**

| Scenario | True distance to goal at "success" (m, worst leg) | AMCL error (m, worst leg) | False successes (>0.5 m off) |
|---|---|---|---|
| A | 0.27 ± 0.0 | 0.15 ± 0.0 | 0 |
| B | 0.32 ± 0.0 | 0.12 ± 0.0 | 0 |
| C | 0.19 ± 0.0 | 0.1 ± 0.0 | 0 |
| C_aisle | 0.16 ± 0.0 | 0.3 ± 0.0 | 0 |
| A_blocked | 0.35 ± 0.0 | 0.16 ± 0.0 | 0 |

Time and distance cover the whole mission (out and back to HOME). Values are mean ± standard deviation.
Min clearance is the closest laser reading during the mission (lowest across trials).
A false success = Nav2 reported the goal reached, but Gazebo shows the robot more than 0.5 m away.
