# Dynamic Obstacle Avoidance — Debug Brief for Claude Code

**Audience:** A fresh Claude Code session with zero prior context on this codebase.
**Goal:** Diagnose why the main robot collides with traffic robots during missions. The user has applied multiple tuning rounds without resolution; we need a rigorous, systematic, evidence-based investigation that proves *which* stage of the Nav2 pipeline is failing, not another tuning iteration.

This document is your complete onboarding. Read it end-to-end before touching anything.

---

## 1. Project context

This is **MECH 650 / EECE 698** (Autonomous Mobile Robots, Spring 2026) at AUB. The final project is an autonomous service robot operating in a simulated town. The user (Suhaib) is on:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress (Ignition)
- Workspace: `~/AMR_MyProject`

**A partner (Boulos) works in parallel on Ubuntu 24.04 + ROS Jazzy + Gazebo Harmonic.** Differences in plugin naming (`gz-sim-*` vs `ignition-gazebo-*`), frame conventions, etc. between the two branches are *expected*, not bugs. Suhaib's `town.world` has mixed naming (`gz-sim-diff-drive-system` for the main robot's diff_drive, `ignition-gazebo-sensors-system` for sensors) and this works on Fortress because the plugin names are aliased. **Do not "fix" the plugin naming.**

### Mission flow
1. User launches `full_system.launch.py use_traffic:=true`.
2. Auto-init-pose publishes a `PoseWithCovarianceStamped` to `/initialpose` 8 s after launch so AMCL localizes without manual RViz clicks.
3. Traffic robots spawn at +15 s.
4. User opens the Python GUI (`mission_console.py`), selects a mission type (grocery / food / fire / medical) and a destination house, clicks Send Mission.
5. The mission executor receives an `ExecuteMission` action goal, decomposes it into three Nav2 `NavigateToPose` calls (source → destination → dock), and serializes them.

### The bug we're chasing
During a mission with traffic robots active, the main robot **collides with traffic robots** — sometimes ending up physically on top of one. Multiple rounds of tuning (critic scales, costmap geometry, inflation, speeds, velocity_smoother timeout, traffic chassis collision sizing) have been attempted. The collision still happens.

We do not yet have *direct evidence* of which stage in the perception → planning → control → actuation pipeline is failing. We've been reasoning from log snippets. **This session's job is to gather direct evidence from a live or post-mortem run, isolate the failure, and prove it.**

---

## 2. Hardware/software environment

### The main robot (`vehicle_blue`)
Defined inline inside `src/amr_gazebo/worlds/town.world`. Key specs:

| Property | Value |
|---|---|
| Chassis box (visual + collision) | 2.0 × 1.0 × 0.5 m |
| Chassis pose | (0, 0, 0.4) relative to model |
| Wheel positions (chassis frame) | (-0.5, ±0.6, 0) |
| Wheel cylinder (visual + collision) | radius 0.4, length 0.2 |
| LiDAR frame (chassis frame) | (0.85, 0, 0.55) |
| LiDAR samples | 360 horizontal, 1 vertical |
| LiDAR update rate | 10 Hz |
| LiDAR range | 0.08 to 30.0 m |
| diff_drive wheel_separation | 1.2 |
| diff_drive wheel_radius | 0.4 |
| diff_drive odom_publish_frequency | 30 Hz |

### Traffic robots
Two of them (red, yellow), defined in `src/amr_traffic/models/traffic_robot_{red,yellow}/model.sdf`. **They were translated from Boulos's Harmonic naming to Fortress naming earlier** (`gz-sim-*` → `ignition-gazebo-*`). Don't touch that translation.

| Property | Value (current) |
|---|---|
| Chassis visual | 2.0 × 1.0 × 0.5 (big) |
| Chassis collision | 1.0 × 0.6 × 0.3 (small — **known mismatch**) |
| Wheel visual | radius 0.2, length 0.1 (we shrunk to match collision in an earlier round) |
| Wheel collision | radius 0.2, length 0.1 |
| Detection bar (visual + collision) | cylinder radius 0.24, length 1.0, at base_link (0, 0, 0.80) |
| LiDAR | gpu_lidar at base_link (0, 0, 0.20), 45 samples, 5 Hz |
| Spawn z | 0.30 (settles to 0.2 in physics) |

A "detection bar" was added on top of the traffic robot's chassis so the main robot's high LiDAR (at world z ≈ 0.95) can see the traffic robot — the chassis alone would be below the LiDAR's scan plane.

### Critical known bugs that have been at least *named*, with status
1. **Static LiDAR TF mismatch** — `navigation.launch.py` publishes a static transform from `vehicle_blue/chassis` → `vehicle_blue/chassis/gpu_lidar` with translation `(0, 0, 1.0)`. The SDF says the LiDAR is at `(0.85, 0, 0.55)` in chassis frame. **This 0.85 m x-offset and 0.45 m z-offset is real**, but not fixed because the saved map was built with this incorrect TF and remapping would be required. Note this and reason about its effects, don't try to fix it now.
2. **Traffic chassis visual/collision mismatch** — visual is 2× larger than collision. Allows the main robot to drive *visually* into the traffic robot while not making physical contact with the smaller collision body. Not yet fixed in committed code.
3. **Possible wheel-climb physics** — main robot wheel radius 0.4 > traffic chassis collision height 0.3, so the main robot's wheels can physically climb the traffic chassis. Hypothesized but unconfirmed by direct evidence.
4. **velocity_smoother `velocity_timeout: 1.0`** — when DWB returns "no valid trajectories," the controller stops publishing cmd_vel. The velocity_smoother holds the last command for up to 1 second before timing out. At 2.5 m/s that's 2.5 m of held-stale-velocity travel. Hypothesized to be the smoking gun. **User may or may not have applied the fix to 0.1 — verify.**
5. **Geometric speed/reaction-distance tension** — at `max_vel_x: 2.5` with a 10-meter local costmap (rolling window radius 5 m), the reaction budget is tight. The math was worked out previously and shows margins around 0.5 m in the best case. Reducing speed to 1.5 m/s gives ~2 m margin. **User wants max speed. Don't tune speed downward without evidence — gather evidence first.**

---

## 3. Repository layout

```
~/AMR_MyProject/
└── src/
    ├── amr_description/           # URDF-style description (legacy, not actively used)
    ├── amr_exploration/           # Autonomous exploration (not used in current mission flow)
    ├── amr_gazebo/                # The town.world, bridge launch, town launch
    │   ├── worlds/town.world      # ← MAIN ROBOT SDF + WORLD CONTENTS
    │   └── launch/
    │       ├── town.launch.py     # starts ign gazebo with town.world
    │       └── bridge.launch.py   # ros_gz parameter_bridge config
    ├── amr_gui/                   # Tkinter mission console (mission_console.py)
    ├── amr_landmarks/             # QR-detection package (EMPTY on Suhaib's branch — lives on Boulos's branch)
    ├── amr_mapping/               # SLAM tooling; not active in mission runs
    ├── amr_mission_executor/      # /execute_mission action server (Python)
    │   ├── amr_mission_executor/mission_node.py   # ← MISSION ORCHESTRATOR
    │   ├── config/landmarks.yaml                  # hardcoded landmark coords (extracted from world)
    │   └── launch/mission_executor.launch.py
    ├── amr_mission_manager/       # Action/service definitions
    │   ├── action/ExecuteMission.action
    │   └── srv/GetLandmarkPosition.srv (defined but not used yet)
    ├── amr_navigation/            # Nav2 wiring
    │   ├── config/nav2_params.yaml                # ← ALL NAV2 TUNING LIVES HERE
    │   ├── launch/full_system.launch.py           # ← TOP-LEVEL BRINGUP
    │   ├── launch/navigation.launch.py            # Nav2 bringup + static LiDAR TF
    │   ├── maps/my_town_map1.{yaml,pgm}           # the static map
    │   └── rviz/                                  # (optional saved RViz config)
    └── amr_traffic/               # Two traffic robots
        ├── amr_traffic/random_walker_node.py
        ├── models/traffic_robot_{red,yellow}/model.sdf
        └── launch/
            ├── spawn_one_traffic_robot.launch.py
            ├── two_traffic_robots.launch.py
            └── traffic_only_demo.launch.py
```

---

## 4. The Nav2 algorithmic pipeline as it should work

For dynamic-obstacle avoidance to succeed, this exact sequence must happen, in order, on time:

1. **LiDAR scan generated.** The Gazebo `gpu_lidar` sensor fires 360 rays at 10 Hz. Returns range/intensity per ray. Frame_id: `vehicle_blue/chassis/gpu_lidar`.
2. **Bridge converts to ROS LaserScan.** The `ros_gz_bridge parameter_bridge` translates `ignition.msgs.LaserScan` to `sensor_msgs/msg/LaserScan` on topic `/scan`.
3. **Static TF publishes the sensor pose.** `static_transform_publisher` claims `(0, 0, 1.0)` from chassis → lidar (KNOWN WRONG; see §2.1).
4. **Obstacle layer consumes the scan.** In `nav2_params.yaml`, the `local_costmap.local_costmap.obstacle_layer` subscribes to `/scan` with `data_type: LaserScan, marking: true, clearing: true, obstacle_max_range: 6.0, raytrace_max_range: 8.0`. Each scan ray's endpoint becomes an occupied cell within the rolling window. Empty space along the ray is cleared. The local costmap is `width: 10, height: 10` (so 10 × 10 m), `rolling_window: true`, `update_frequency: 15.0` Hz.
5. **Inflation layer pads obstacles.** `inflation_radius: 1.0`, `cost_scaling_factor: 3.0`. Around each obstacle cell, cost decays from LETHAL (254) at distance 0 to INSCRIBED_INFLATED (253) within `inscribed_radius` (computed from the footprint's inscribed circle = 0.75 m for the rectangle `[[-1.05,-0.75],[-1.05,0.75],[1.05,0.75],[1.05,-0.75]]`), then exponentially to 0 at distance 1.0 m.
6. **DWB samples candidate trajectories.** `controller_frequency: 20.0` Hz. At each tick, DWB generates `vx_samples * vtheta_samples` = `30 * 20 = 600` candidate `(vx, ω)` pairs, simulates each forward over `sim_time: 1.2` s at constant velocity, producing 600 arcs.
7. **Critics score each trajectory.**
   - `BaseObstacle.scale: 1.0` — cost at trajectory center pose
   - `ObstacleFootprint.scale: 1.0` — max cost over the footprint polygon at each pose
   - `PathDist.scale: 24.0`, `PathAlign.scale: 24.0` — distance to / alignment with global path
   - `GoalDist.scale: 24.0`, `GoalAlign.scale: 24.0` — distance to / alignment with goal
   - `Oscillation` — penalizes oscillating behavior
   The lowest-cost trajectory wins.
8. **Winning (vx, ω) becomes the controller's cmd_vel.** Published on `/cmd_vel` (after going through `velocity_smoother`).
9. **velocity_smoother enforces accel/decel limits.** `max_velocity: [2.5, 0.0, 2.5]`, `max_accel: [4.0, 0.0, 4.0]`, `max_decel: [-4.0, 0.0, -4.0]`, **`velocity_timeout: 1.0`** (this is the suspected bug). Smoothed cmd_vel is published.
10. **Bridge forwards cmd_vel to Gazebo.** `ros_gz_bridge` translates `geometry_msgs/msg/Twist` → `ignition.msgs.Twist` on `/cmd_vel`. The diff_drive plugin (`gz-sim-diff-drive-system`) subscribes to `cmd_vel` and applies wheel velocities.
11. **Wheels rotate, robot moves.** Gazebo physics resolves contacts and wheel torques.

**Any one of these stages can be the failure point.** The whole point of this debug session is to *instrument each stage* and prove which one is breaking.

---

## 5. Failure hypotheses ranked by prior probability

These are *hypotheses to test*, not conclusions. Test each one with the procedure in §7.

### H1 — velocity_smoother holds stale velocity when DWB fails
- Mechanism: DWB returns "no valid trajectories." controller_server stops publishing cmd_vel. velocity_smoother holds the last command (potentially 2.5 m/s forward) for `velocity_timeout: 1.0` second. The robot coasts straight through whatever was in front of it.
- **Verification path:** check the current value of `velocity_timeout`. Echo `/cmd_vel` during a collision event and look for high forward velocity persisting when DWB logs "no valid trajectories."
- **Quick fix if confirmed:** `velocity_timeout: 0.1`.

### H2 — Traffic chassis visual/collision mismatch + wheel-climb
- Mechanism: traffic chassis visual is 2.0 × 1.0 × 0.5 but collision is 1.0 × 0.6 × 0.3. Visually the robots appear to "drive into" each other while collision bodies haven't touched. When they do contact, the main robot's 0.4 m wheels can climb the 0.3 m chassis (because wheel_radius > obstacle_height).
- **Verification path:** observe a collision event in Gazebo. If the main robot ends up tilted on top of the traffic chassis, this is the bug. If the main robot bounces back or is stopped flat, it isn't.
- **Quick fix if confirmed:** set traffic chassis collision to `2.0 × 1.0 × 0.5` with pose offset `(0, 0, 0.10)` to keep it above ground.

### H3 — Local costmap not actually receiving / marking traffic robots
- Mechanism: scan frame_id wrong, TF chain broken, scan rate too low, max_obstacle_height filtering out the scan, or some other reason the traffic robots don't appear as occupied cells in the local costmap.
- **Verification path:** RViz with `/local_costmap/costmap` display, drive main robot manually toward a traffic robot, see if cells light up. Or `ros2 topic echo /local_costmap/costmap_updates`. Or check costmap occupancy via service.
- **Critical:** if this is the bug, no amount of critic/inflation tuning will help.

### H4 — TF mismatch causing costmap-frame errors
- Mechanism: the static TF claiming the LiDAR is at chassis `(0, 0, 1.0)` while it's actually at `(0.85, 0, 0.55)`. Obstacles in the costmap appear 0.85 m closer to chassis-x than reality. This is documented; the question is whether it's *catastrophic* for avoidance (it makes the robot more conservative, not less — but maybe in combination with other things it triggers the failure).
- **Verification path:** compute the actual obstacle position in the costmap when a traffic robot is at a known world position, vs. what AMCL/TF should place it at.

### H5 — DWB scoring weights still wrong despite multiple iterations
- Mechanism: PathDist/PathAlign dominate over BaseObstacle/ObstacleFootprint, so trajectories along the path through an obstacle outrank trajectories deviating around it.
- **Verification path:** review the actual critic costs DWB computes (DWB can log them with `debug_trajectory_details: true`).

### H6 — High speed + tight reaction distance, no specific bug
- Mechanism: at 2.5 m/s closing 3.5+ m/s, the geometry is fundamentally near the edge of what reactive avoidance can do at all.
- **Verification path:** run the same configuration at 1.0 m/s. If avoidance works, it's a speed problem; if not, it's a different bug.

---

## 6. Files to read first

Read in this order, no exceptions. Each file is small enough to read whole.

1. `src/amr_navigation/config/nav2_params.yaml` — the entire Nav2 configuration. Read every block. Understand every parameter. This is the central truth.
2. `src/amr_navigation/launch/full_system.launch.py` — top-level bringup. Look especially at the traffic include's `launch_arguments` (spawn positions, bounds, speeds).
3. `src/amr_navigation/launch/navigation.launch.py` — note the static_transform_publisher with `(0, 0, 1.0)` translation.
4. `src/amr_gazebo/worlds/town.world` lines 50–250 (the main robot's SDF). Note: chassis pose, lidar_frame pose, LiDAR sensor config, diff_drive plugin block. The world is large (~1860 lines), only the vehicle_blue section is relevant here.
5. `src/amr_gazebo/launch/bridge.launch.py` — confirm `/scan`, `/cmd_vel`, `/odom`, `/tf`, `/clock` are all bridged.
6. `src/amr_traffic/models/traffic_robot_red/model.sdf` (yellow is identical except color) — the chassis visual/collision mismatch, the bar, the wheel sizes.
7. `src/amr_traffic/launch/spawn_one_traffic_robot.launch.py` and `two_traffic_robots.launch.py` — confirm what gets spawned and how.
8. `src/amr_traffic/amr_traffic/random_walker_node.py` — the traffic robot FSM logic (this isn't the avoidance bug source, but it determines where traffic robots are during a mission).
9. `src/amr_mission_executor/amr_mission_executor/mission_node.py` — the mission executor. Look for how the mission is decomposed and how Nav2 goals are sent.

After reading these, you should be able to draw the full data flow from LiDAR scan → costmap → controller → cmd_vel → wheels in your head.

---

## 7. Diagnostic procedure — do this in order, do not skip

Each step gathers a specific piece of evidence. Do not modify any code until at least steps 1–6 have produced concrete observations. The goal is *isolating the failure*, not patching forward.

### Step 1 — Confirm the current parameter state

Several fixes have been suggested across rounds. The user may or may not have applied them. Confirm the actual current state:

```bash
# Read the user's actual configured values
grep -E "velocity_timeout|failure_tolerance|max_vel_x|max_speed_xy|inflation_radius|footprint|update_frequency|width:|height:" \
  ~/AMR_MyProject/src/amr_navigation/config/nav2_params.yaml

# Read the actual current full_system launch (especially traffic args)
grep -E "bounds_radius|forward_speed|robot1_|robot2_" \
  ~/AMR_MyProject/src/amr_navigation/launch/full_system.launch.py
```

Compare to the "ideal" values:
- `velocity_timeout: 0.1` (was 1.0; fix from prior round)
- `failure_tolerance: 0.1` (was 0.3)
- `max_vel_x: 2.5`, `max_speed_xy: 2.5` (user wants high speed)
- `inflation_radius: 1.0` (must be >= inscribed_radius 0.75)
- `footprint`: `"[[-1.05, -0.75], [-1.05, 0.75], [1.05, 0.75], [1.05, -0.75]]"`
- `update_frequency: 15.0`, `width: 10`, `height: 10`
- `bounds_radius: "5.0"`, `forward_speed: "0.7"` or `"0.8"`

Record which fixes are applied and which aren't. This alone may explain things.

### Step 2 — Confirm the topic graph at runtime

Launch the system once and let it settle (don't send a mission yet):

```bash
ros2 launch amr_navigation full_system.launch.py use_traffic:=true
```

In a second terminal, after ~20 seconds:

```bash
# Verify scan topic is alive and at expected rate
ros2 topic hz /scan
# Expected: ~10 Hz

# Verify scan frame_id
ros2 topic echo /scan --once --field header.frame_id
# Expected: vehicle_blue/chassis/gpu_lidar

# Verify the local costmap is publishing
ros2 topic hz /local_costmap/costmap
# Expected: ~2 Hz (publish_frequency)

# Verify cmd_vel publishers
ros2 topic info /cmd_vel
# Expected: publishers = velocity_smoother and ros_gz_bridge

# Verify the TF chain reaches LiDAR
ros2 run tf2_ros tf2_echo vehicle_blue/chassis vehicle_blue/chassis/gpu_lidar
# Expected: translation (0, 0, 1.0) — this is the static TF; document the known mismatch

# Verify map→odom is being published by AMCL
ros2 run tf2_ros tf2_echo map vehicle_blue/odom
# Expected: a transform exists (initially zero, updates as AMCL refines)
```

If any of these fail, **stop and report it before continuing.** This is upstream of the avoidance question entirely.

### Step 3 — Visually verify the LiDAR sees traffic robots

This is the single most critical empirical question. If the LiDAR doesn't see the traffic robots, no avoidance can ever work.

Launch with RViz enabled:
```bash
ros2 launch amr_navigation full_system.launch.py use_traffic:=true use_rviz:=true
```

In RViz, add:
- Fixed Frame: `map`
- `LaserScan` display, topic `/scan`, decay time 1.0
- `Map` display, topic `/local_costmap/costmap`, color scheme `costmap`
- `Map` display, topic `/global_costmap/costmap`, color scheme `costmap`, topic durability `transient_local`
- `Path` display, topic `/plan` (global plan)
- `Path` display, topic `/local_plan` (local plan from controller)
- `PolygonStamped` display, topic `/local_costmap/published_footprint`

Send a mission. Watch carefully:
- Do laser scan dots appear on traffic robots when they're in front of the main robot?
- Do the corresponding cells in `/local_costmap/costmap` light up?
- Does the global path (`/plan`) avoid the traffic robot's costmap cells, or does it cut straight through them?
- Does the local path (`/local_plan`) differ from the global path when a traffic robot is in the way?

**Save screenshots** at the moment of collision attempt. This is the most informative single artifact in the entire debug.

### Step 4 — Capture cmd_vel during a collision event

In a terminal during a mission:
```bash
ros2 topic echo /cmd_vel > /tmp/cmd_vel_log.txt
```

Run a mission that has historically caused a collision. Stop the echo (Ctrl-C) right after the collision.

Then:
```bash
# Find what was published right when DWB started having trouble
grep -B 2 -A 2 "x: 2" /tmp/cmd_vel_log.txt | head -50
```

Look for sequences where `linear.x` stays near 2.5 m/s for multiple consecutive messages even when DWB logs "no valid trajectories" in the system log. If that pattern is visible, **H1 (velocity_timeout) is confirmed.**

### Step 5 — Capture the controller_server log during a collision

Launch the system and tee the log to a file:
```bash
ros2 launch amr_navigation full_system.launch.py use_traffic:=true 2>&1 | tee /tmp/run.log
```

Run a mission that causes a collision. Then post-mortem:
```bash
# Count "no valid trajectories" warnings
grep -c "No valid trajectories" /tmp/run.log

# See if the controller failed and recovered
grep -E "No valid trajectories|Resulting plan has 0 poses|Passing new path|Reached the goal|aborted|failed" /tmp/run.log
```

The pattern of these logs around the collision time tells you whether DWB was failing-and-recovering (H1 territory) or working normally (some other failure).

### Step 6 — Compare the SDF claims to the running physics

Read `town.world` lines 52–140 and `traffic_robot_red/model.sdf` end-to-end. Then in Gazebo's Entity Tree (sidebar in the Gazebo GUI), inspect:
- The main robot's entity tree — confirm collision shapes by clicking each link.
- A traffic robot's entity tree — same.

If the traffic robot's chassis collision is `1.0 × 0.6 × 0.3` (current), this is H2 territory. Make a note.

### Step 7 — Quantitative reaction-budget calculation

After observing the above, do the math for the **observed** speeds (echo `/cmd_vel` to find the actual max forward velocity during the mission, not the configured value — they may differ if the diff_drive plugin or velocity_smoother caps things):

```
Closing speed = main_robot_max_vx + traffic_max_vx
Local costmap radius = width / 2
Reaction latency ≈ 1/scan_rate + 1/costmap_update + 1/controller_freq
Distance closed during latency = closing_speed × reaction_latency
Effective reaction distance = local_costmap_radius - distance_closed_during_latency
Stopping distance = main_robot_max_vx² / (2 × decel_lim_x)
Stopping time = main_robot_max_vx / decel_lim_x
Distance closed by traffic during stop = traffic_max_vx × stopping_time
Required clearance = main_robot_half_length + inscribed_radius
Margin = effective_reaction_distance - stopping_distance - distance_closed_during_stop - required_clearance
```

If `Margin < 0.2 m`, **the system has no real safety margin** and any glitch will produce a collision — confirming H6 even with no specific bug.

### Step 8 — A controlled experiment to differentiate hypotheses

Run the same mission three times with these one-variable changes:

**Run A — Drop main robot speed to 1.0 m/s.** Edit `nav2_params.yaml`: `max_vel_x: 1.0`, `max_speed_xy: 1.0`, matching `velocity_smoother.max_velocity[0]: 1.0`. Rebuild, run. If collisions stop, the problem is geometric/timing (H1, H6) and high speed exposes it. If they continue at 1.0 m/s, the problem is structural (H3, H4 — costmap or perception).

**Run B — Disable traffic robots entirely.** `use_traffic:=false`. Run a normal mission. Confirm it completes cleanly. If it doesn't, something is wrong with the basic navigation, not avoidance.

**Run C — Stationary traffic robots.** Modify `random_walker_node.py` to publish `cmd_vel.linear.x = 0` always (or just set `forward_speed: "0.0"` in the launch). Spawn them in the middle of a planned path. Run a mission. The main robot should easily avoid stationary obstacles. If it doesn't, the issue is *perception* (H3, H4), not reaction time.

These three runs together give you 90% of the diagnostic signal.

---

## 8. What we already know

- The robot **does** complete missions to landmarks without traffic. So the basic Nav2 pipeline works end-to-end.
- The robot **does** sometimes complete missions with traffic; collisions are intermittent, not 100%.
- `controller_server` logs "No valid trajectories out of 629!" repeatedly during collision events, mixed with `Passing new path to controller` messages.
- The robot has been observed *physically on top of* a traffic robot's chassis in Gazebo (H2-style wheel-climb), implying physical contact does happen, not just visual overlap.
- The current state of `nav2_params.yaml` in the repo: `max_vel_x: 2.5`, `inflation_radius: 1.0`, `width: 10`, `velocity_timeout: 1.0` (likely — needs verification), `BaseObstacle.scale: 1.0`, `ObstacleFootprint.scale: 1.0`, `PathDist.scale: 24.0`.

---

## 9. Constraints on your investigation

- **Don't tune.** This session is for diagnosis only. Tuning has been tried and isn't producing convergence. Identify the root cause first.
- **Don't fix the TF mismatch.** The map was built with the wrong TF and changing it requires remapping. Reason about its effects but leave it alone.
- **Don't fix the plugin naming.** Mixed `gz-sim-*` and `ignition-gazebo-*` is intentional and working.
- **Don't reach for MPPI or SmacPlanner.** Stay with DWB and NavfnPlanner. Switching controllers is a big refactor and we want to understand the current system's failure first.
- **Don't disable critics or change weights without evidence.** Same reason.
- **Do investigate broadly first.** Reading lots of files, running lots of diagnostic commands, gathering evidence — all good. Even reading parts of the codebase that seem unrelated. The bug may be somewhere we haven't looked.
- **Do reason from evidence.** When you form a hypothesis, design a check that would falsify it before deciding it's correct.

---

## 10. Deliverables

When you finish:

1. A clear statement of which hypothesis is confirmed (or a new hypothesis if you found one).
2. The specific evidence — log lines, screenshots, parameter values — that confirms it.
3. The specific lines of code or configuration that need to change to fix the root cause.
4. A *proposed* fix (don't apply it unilaterally — let the user review). Include the exact diff or before/after.
5. A predicted effect: "after this fix, the robot should X." This is so we can verify the fix actually addresses the diagnosis.

Don't conclude "it must be tuning" or "lower the speed" without evidence. We've been there.

---

## 11. Reading the user's intent

The user (Suhaib) is a graduate student running this for a class project with a partner. He values direct technical depth, doesn't want filler, will push back if something is wrong. He's been frustrated by multi-round tuning that hasn't converged. He wants a *root-cause* answer, not another round of tweaks. He's specifically asked you to do max-effort debugging.

Don't sugar-coat. If your investigation reveals that the system has fundamental geometric/speed limits that no amount of tuning will fix, say so. If it reveals a specific bug, fix it precisely. Either is acceptable; what's not acceptable is vague tuning suggestions without diagnosis.

---

## 12. Final note

If after the diagnostic procedure in §7 you genuinely cannot localize the failure, that itself is a finding worth reporting. Describe what you tried, what each step showed, and what's still unexplained. We'd rather know what's *not* the bug than have a confident-sounding but wrong answer.

Good luck. Read the files in §6, then run the diagnostics in §7, then report.
