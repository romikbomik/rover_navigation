# Path-following controller report

## What a PID is

A PID controller turns a scalar error \(e\) into a command \(u\):

\[
u = K_p\, e + K_i \int e\,dt + K_d \frac{de}{dt}
\]

- **P** reacts to the current error — larger offset, stronger correction.
- **I** accumulates a persistent offset and removes steady-state bias (for example
  always sitting a few centimetres right of the line). The integral is clamped
  so it cannot wind up forever.
- **D** reacts to how fast the error is changing and damps the weaving that P
  alone would cause.

## Architecture

`ArduroverController` is an abstract base: it arms the rover, switches to
GUIDED, sets `mav_frame` to `BODY_NED`, and publishes twists. `ControllerPID`
overrides `Control()` with the tracking law. A small `Pid` class holds the
scalar math; `ReferencePath` owns projection onto the recorded polyline.

## Error and commands

Each 20 Hz tick:

1. Project the rover onto the path → closest point \(p^*\), path heading
   \(\psi_{path}\), signed cross-track \(e_{ct}\), remaining length.
2. Heading error \(e_\psi = \mathrm{wrap}(\psi_{path} - \psi_{rover})\).
3. Blended error \(e = e_\psi + k_{ct}\, e_{ct}\) (radians).
4. Yaw rate \(\omega = \mathrm{clamp}(\mathrm{PID}(e),\, -\omega_{max},\, \omega_{max})\).
5. Speed (no PID — ArduRover already closes the speed loop):
   - cruise `max_speed` when roughly aligned
   - `pivot_speed` when \(|e_\psi| >\) `pivot_angle`
   - linear ramp to zero inside `slow_radius` of the end

Published on `/mavros/setpoint_velocity/cmd_vel_unstamped`:

| Field | Meaning |
| --- | --- |
| `linear.x` | Forward speed \(v\) (m/s), body frame |
| `angular.z` | Yaw rate \(\omega\) (rad/s), ROS FLU (positive left) |

## Parameters

| Parameter | Default | Role |
| --- | ---: | --- |
| `pid_kp` | 1.5 | Proportional gain |
| `pid_ki` | 0.0 | Integral gain |
| `pid_kd` | 0.2 | Derivative gain |
| `pid_integral_limit` | 0.5 | Anti-windup clamp on the integral |
| `cte_gain` | 1.0 | Metres of CTE → radians of blended error |
| `max_speed` | 1.0 | Cruise speed (m/s) |
| `max_yaw_rate` | 1.0 | Yaw-rate limit (rad/s) |
| `pivot_angle` | 1.0 | Misalignment that forces crawl (rad) |
| `pivot_speed` | 0.15 | Crawl speed (m/s) |
| `slow_radius` | 1.5 | Distance at which arrival ramp starts (m) |
| `goal_tolerance` | 0.25 | Stop when remaining path is under this (m) |

Tuning followed the planned recipe: start with P on the straight path, add D to
dampen weave, leave I at zero (no steady sideways bias showed up).

## Scores

```bash
ros2 launch ardurover_nav control.launch.py gz_gui:=false \
  path_file:=/home/developer/ardurover_navigation/paths/<path>
```

| Path | Score | Completion | RMS CTE (m) | Max CTE (m) |
| --- | ---: | ---: | ---: | ---: |
| `0-drive-straight.path` | 95.0 | 0.97 | 0.010 | 0.045 |
| `1-drive-with-turns.path` | 91.1 | 0.98 | 0.033 | 0.108 |
| `2-complicated.path` | 68.9 | 0.99 | 0.112 | 0.839 |

All three finished with `reason: goal_reached`. The complicated path overshoots
tight corners because pure feedback has no lookahead; lowering speed trades
time for accuracy if a higher score is needed there.
