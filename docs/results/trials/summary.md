# Delivery trials: final-20261107-2259

15 trials. Behaved as expected: **87%**

| Scenario | n | Expected | Outcomes | As expected | Time (s) | Distance (m) | Nav2 recoveries | Retries | Min clearance (m) |
|---|---|---|---|---|---|---|---|---|---|
| A | 3 | success | success 3 | 100% | 95.03 ± 1.36 | 16.82 ± 0.29 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.61 |
| B | 3 | success | success 3 | 100% | 96.2 ± 1.08 | 16.92 ± 0.14 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.64 |
| C | 3 | success | success 3 | 100% | 87.27 ± 0.76 | 14.83 ± 0.03 | 0.0 ± 0.0 | 0.0 ± 0.0 | 1.06 |
| C_aisle | 3 | success | success 3 | 100% | 200.23 ± 21.83 | 32.74 ± 2.47 | 2.0 ± 1.0 | 0.0 ± 0.0 | 0.42 |
| A_blocked | 3 | partial | failed 1, partial 1, success 1 | 33% | 169.33 ± 61.09 | 14.56 ± 3.48 | 28.0 ± 39.89 | 2.0 ± 2.0 | 0.16 |

**Ground truth check (simulation only, from Gazebo):**

| Scenario | True distance to goal at "success" (m, worst leg) | AMCL error (m, worst leg) | False successes (>0.5 m off) |
|---|---|---|---|
| A | 0.26 ± 0.05 | 0.14 ± 0.02 | 0 |
| B | 0.29 ± 0.02 | 0.18 ± 0.03 | 0 |
| C | 0.23 ± 0.02 | 0.09 ± 0.02 | 0 |
| C_aisle | 0.15 ± 0.01 | 0.11 ± 0.01 | 0 |
| A_blocked | 0.73 ± 0.52 | 1.55 ± 1.28 | 1 |

Time and distance cover the whole mission (out and back to HOME). Values are mean ± standard deviation.
Min clearance is the closest laser reading during the mission (lowest across trials).
A false success = Nav2 reported the goal reached, but Gazebo shows the robot more than 0.5 m away.
