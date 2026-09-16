# Thema 1 – Automatische Kartierung on Raupy

**Deadline:** presentation by **2026-09-25** (15 min). Also required: commented code, and a
README in git that says who did what ("Wer hat was gemacht").

**Approach:** use the library `mertgulerx/frontier_exploration_ros2` as is. We don't implement
the lecture's own scoring formula. Our own work is:
- integrating the library on Raupy (SLAM, Nav2, explorer config, launch files)
- a supervisor node that stops the run and saves the map

## Status (2026-09-16)

Phases 0–4 are implemented and **tested offline only** (no robot):
- `colcon build` works for both packages; tests: library 141/141, `raupy_exploration` 18/18 (+ lint).
- End-to-end run against `fake_raupy.py`, a kinematic fake robot with Raupy's exact interfaces
  (TwistStamped `/cmd_vel`, `/odometry/filtered`, TF `odom→base_link`, `laser` mounted at
  yaw −90°, best-effort `/scan`) in a 14 × 9 m floor plan:
  - 24 frontier goals
  - `exploration_complete` after 184 s, with 124 of 126 m² known
  - supervisor STOP → Nav2 cancel → zero velocity → map `.pgm/.yaml` + `.posegraph/.data`
    + `_summary.yaml` all succeeded
- Not yet tested: the `laser_filters` box filter (package not installed; the offline run used
  the `scan_relay.py` pass-through via `use_scan_filter:=false`), and **anything on the real
  robot** (Wi-Fi latency, real tuning).

Offline test (from the repo root):
```bash
source install/setup.bash && export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run raupy_exploration fake_raupy.py &
ros2 launch raupy_exploration exploration.launch.py use_scan_filter:=false max_duration_s:=360.0 map_output_dir:=$PWD/maps_offline
```

Real robot (Phase 5):
```bash
ssh husarion@raupy.roblab.cs.hs-fulda.de rosbot-lidar.sh start
source scripts/raupy_env.sh
ros2 launch raupy_exploration exploration.launch.py
```

## 1. Task (`folien/Robotik_SoSe2025_Projekt_compressed-2.pdf`, slide 509)

| Requirement | Covered by |
|---|---|
| Build a map of the surroundings on its own, step by step | slam_toolbox (online async) on the Raupy lidar |
| Drive to poses where new information is most likely | `frontier_exploration_ros2`: frontier detection, scoring by information gain and path cost (MRTSP), goals sent to Nav2 |
| Target the borders ("Kanten") between known and unknown space | Frontiers = known-free cells next to unknown cells |
| Stop when no new information comes in, **or** after a time limit | Library completion event **plus** our `exploration_supervisor` (map-growth stagnation and time budget) |
| Save the map, then stop | `exploration_supervisor` calls map_saver and slam_toolbox serialization, then stops the robot |

In the presentation, relate the library to the lecture's SPLAM slides (510–512) in words:

| Lecture | Library |
|---|---|
| Riss-Polygon edges | Frontiers |
| IG(x) | Frontier size / information gain |
| Distance and turn-angle penalties | MRTSP path cost and heading term |

No code for this.

## 2. Robot facts – Raupy (inspected 2026-09-16)

Raw dumps are in `wf/raupy_info.txt`, `wf/raupy_info2.txt` and `wf/raupy_live.txt`
(gitignored, since they contain shell history). They were generated with `wf/raupy_inspect.sh`.

| Item | Value |
|---|---|
| Hardware | Husarion **ROSbot 2 PRO**: CORE2 STM32 + UP Board (Atom, 4 cores, 3.2 GB RAM), 4-wheel skid steer |
| Access | `ssh husarion@raupy.roblab.cs.hs-fulda.de` (10.33.130.103), key auth set up |
| OS / ROS | Ubuntu 24.04, ROS 2 **Jazzy**, native install (no Docker). Driver workspace `~/rosbot_ws` (husarion `rosbot_ros`) |
| Network | Wi-Fi `HSFD-M2M` on 2.4 GHz. The robot README budgets **2–3 MB/s** usable throughput |
| ROS domain | **103**: 100 + the last two digits of the IP. Recomputed at every service start, so it **changes with a new DHCP lease** |
| RMW | `rmw_fastrtps_cpp`, and the robot also sets `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` |
| Always running | `microros.service` (firmware bridge) and `rosbot.service` (ros2_control diff drive, IMU, EKF, joystick teleop, laser filter) |
| On demand | `rosbot-lidar.sh start|stop|status` (RPLIDAR A3). The lidar is off by default |
| Drive command | `/cmd_vel` as **`geometry_msgs/TwistStamped`**. A plain `Twist` is silently ignored. `cmd_vel_timeout` is 0.5 s, so the robot stops if commands stop |
| Controller limits | linear ±1.0 m/s, acc 1.0 m/s²; angular ±3.14 rad/s, acc 4.0 rad/s² |
| Odometry | Wheel odometry `/odometry/wheels` plus IMU go into the EKF, which publishes **`/odometry/filtered`** and TF `odom → base_link` (~14 Hz) |
| Base frame | **`base_link`**. There is **no `base_footprint`** |
| TF tree | `odom → base_link → body_link → cover_link → rplidar_link → laser`. `base_link → laser`: xyz (0.020, 0, 0.131), **yaw −90°** |
| Lidar | RPLIDAR A3, **360°**, **1800 beams**, 0.05–25 m, 10 Hz, frame `laser`. Topic **`/scan`** |
| `/scan_filtered` | Should come from the robot's `laser_filters` node, but **published nothing** during inspection. We won't use it (see Phase 1) |
| Chassis filter box | In `base_link`: x −0.17..0.10, y −0.12..0.12, z 0..0.2 (robot's `rosbot_utils` config) |
| Footprint | Body 0.197 × 0.150 m, overall ≈ 0.20 × 0.235 m. Earlier Raupy Nav2 config used `robot_radius 0.18`, inflation 0.30 |
| Reference configs on the robot | `~/Nav2/nav2_params_raupy.yaml` (RPP, stamped cmd_vel), `/etc/rosbot/slam-params.yaml` (res 0.05). Values get copied into our repo, nothing is run from there |

**Verified from the laptop** (domain 103, fastrtps):
- all topics are discovered
- `/odometry/filtered` arrives at 13.6 Hz
- `/scan` arrives at ~12.9 Hz

## 3. Architecture

```
 Raupy (robot, nothing installed by us)        Laptop (this repo)
 ──────────────────────────────────────        ───────────────────────────────────────────────
 rosbot.service (boot)                         laser_filters (box filter)  /scan → /scan_clean
   diff drive, EKF → TF odom→base_link         slam_toolbox (online async) /scan_clean → /map, TF map→odom
   /odometry/filtered, /cmd_vel (Stamped)      Nav2: planner, RPP controller, costmaps, behaviors,
 rosbot-lidar.sh start                               velocity_smoother → /cmd_vel (TwistStamped)
   /scan (360°, 10 Hz)                         frontier_explorer (library)
                                               exploration_supervisor (ours): stop + save map
                                               RViz
                  ◄──── Wi-Fi · ROS_DOMAIN_ID=103 · rmw_fastrtps_cpp ────►
```

- Nothing is copied to or built on the robot. The robot runs only its preinstalled services.
- Only `/scan` (~0.3 MB/s), odometry and TF go to the laptop, and `/cmd_vel` comes back.
  That is well inside the Wi-Fi budget.
- **Fallback** if the laptop CPU or Wi-Fi latency is a problem: start SLAM on the robot with
  `rosbot-slam.sh`, which is preinstalled. Everything else stays on the laptop.

## 4. Libraries

| Package | Purpose | Status on laptop |
|---|---|---|
| `frontier_exploration_ros2` v1.6.1 (git submodule) | Frontier exploration | to add |
| `ros-jazzy-laser-filters` | Remove chassis returns from `/scan` | **install** |
| `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup` | Navigation incl. RPP controller, map_saver | installed |
| `ros-jazzy-slam-toolbox` | SLAM | installed |
| `ros-jazzy-teleop-twist-keyboard` | Manual driving / safety | installed |
| `ros-jazzy-rviz2`, `ros-jazzy-nav2-rviz-plugins`, `ros-jazzy-tf2-tools` | Visualisation / debugging | installed |
| `ros-jazzy-rosbag2` | Recording demo runs | installed |

The library's own dependencies (rclcpp_action, nav2_msgs, visualization_msgs, tf2_ros,
python3-yaml, gtest) are part of the installed ROS / Nav2 stack. Confirm with rosdep:

```bash
sudo apt install ros-jazzy-laser-filters
rosdep install --from-paths ros2_ws/src --ignore-src -y     # after adding the submodule
```

## 5. Work plan

### Phase 0 – Repo & environment (≈0.5 day, by 2026-09-17)
- [ ] `.gitignore` (done): creds, raw dumps, `build/ install/ log/`.
- [ ] Layout: `ros2_ws/src/`. Build with `colcon build --symlink-install` from the repo root.
- [ ] `git submodule add https://github.com/mertgulerx/frontier_exploration_ros2 ros2_ws/src/frontier_exploration_ros2`
      and pin it to the release tag/commit of v1.6.1. Don't fork, since we don't patch it.
- [ ] Create package `raupy_exploration` (ament_cmake + Python node or C++):
      `config/ launch/ rviz/ scripts/ src/ maps/`.
- [ ] `scripts/raupy_env.sh` (source it before every session):
  - `source /opt/ros/jazzy/setup.bash` and the workspace overlay
  - `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
  - `ROS_DOMAIN_ID` read live from the robot:
    `ssh husarion@raupy.roblab.cs.hs-fulda.de 'cut -d= -f2 /etc/default/rosbot-domain'`
- [ ] Install `ros-jazzy-laser-filters`, run rosdep, build, and run the library's tests
      (`colcon test --packages-select frontier_exploration_ros2`).

**Done when:** the workspace builds cleanly, the tests pass, and
`ros2 topic list` shows the robot topics after sourcing `raupy_env.sh`.

### Phase 1 – Sensor pipeline + SLAM on the real robot (≈1 day, by 2026-09-18)
- [ ] `config/scan_filter.yaml`: `laser_filters/LaserScanBoxFilter` in `base_link`, with the
      box from the robot config. Add a small margin, e.g. x −0.19..0.12, y ±0.14, z 0..0.25.
      `scan_to_scan_filter_chain` remaps `scan → /scan`, `scan_filtered → /scan_clean`.
      Use sensor-data QoS if needed.
- [ ] `config/slam_raupy.yaml`, based on `/etc/rosbot/slam-params.yaml`:
  - `scan_topic: /scan_clean`, `base_frame: base_link`, `odom_frame: odom`, `map_frame: map`
  - `mode: mapping`, `resolution: 0.05`, `max_laser_range: 12.0` (indoor, less noise)
  - `minimum_travel_distance/heading: 0.2`, `map_update_interval: 2.0`
  - `transform_timeout: 0.5`, `tf_buffer_duration: 30.0` (TF comes over Wi-Fi)
- [ ] `launch/slam.launch.py`: scan filter + slam_toolbox `online_async_launch.py` with our params.
- [ ] `rviz/exploration.rviz`:
  - fixed frame `map`
  - Map display with durability *Transient Local*
  - LaserScan `/scan_clean`, TF, RobotModel
  - global/local costmaps
  - frontier markers `/explore/frontiers`, selected frontier `/explore/selected_frontier`
- [ ] Test: on the robot run `rosbot-lidar.sh start`. On the laptop run the slam launch and
      drive with
      `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link`
      (reduce speed first).

**Done when:**
- a room/corridor map builds cleanly
- there is no chassis ring at the origin
- `map → odom` stays stable while turning
- laptop CPU is OK

### Phase 2 – Nav2 for Raupy (≈1–1.5 days, by 2026-09-19)
- [ ] `config/nav2_raupy.yaml`, starting from `~/Nav2/nav2_params_raupy.yaml` and nav2_bringup defaults:
  - **Frames / topics:**
    - `global_frame: map`, `robot_base_frame: base_link` everywhere
    - `odom_topic: /odometry/filtered` (bt_navigator, velocity_smoother)
    - `enable_stamped_cmd_vel: true` on controller_server, behavior_server,
      velocity_smoother and collision_monitor if used
  - **Controller:** Regulated Pure Pursuit
    - `desired_linear_vel: 0.25`, `lookahead_dist` ~0.5
    - `use_rotate_to_heading: true`, `rotate_to_heading_angular_vel: 0.8`
    - `transform_tolerance: 0.5`
  - **Velocity smoother:** `max_velocity [0.25, 0, 1.0]`, `max_accel [0.8, 0, 1.5]`.
  - **Costmaps:**
    - `robot_radius: 0.18`, `inflation_radius: 0.35`, `cost_scaling_factor: 4.0`,
      `resolution: 0.05`
    - observation source `/scan_clean`, raytrace range 6 m, obstacle range 5 m
    - local: rolling 3×3 m in `odom`
    - global: `static_layer` on `/map` (`map_subscribe_transient_local: true`), obstacle
      layer and inflation, `track_unknown_space: true`
  - **Planner:** NavFn or Smac2D with `allow_unknown: true`, because frontier goals lie at
    the edge of unknown space.
  - **Progress checker:** `required_movement_radius: 0.3`, `movement_time_allowance: 15.0`.
  - **Goal checker:** `xy_goal_tolerance: 0.25`, `yaw_goal_tolerance: 0.5` (yaw at frontiers
    doesn't matter much).
- [ ] `launch/navigation.launch.py`: nav2_bringup `navigation_launch.py` with our params. No
      AMCL and no map_server, since slam_toolbox provides `/map` and `map → odom`.
- [ ] Test: with SLAM running, send goals in RViz (Nav2 Goal tool), including a goal behind
      an obstacle and one near a wall.

**Done when:** goals are reached without collision, recovery behaviours (spin, backup) work,
and there's no oscillation.

### Phase 3 – Frontier explorer (≈1 day, by 2026-09-20)
- [ ] `config/explorer_raupy.yaml` (copy of the library `params.yaml`, changes only):
  - **Topics / frames:**
    - `robot_base_frame: base_link`
    - `map_topic: /map`, `costmap_topic: /global_costmap/costmap`,
      `local_costmap_topic: /local_costmap/costmap`
  - **Control:**
    - `control_service_enabled: true` (the supervisor needs it)
    - `completion_event_enabled: true`, `return_to_start_on_complete: false`
    - `autostart: true`, or `false` with the supervisor starting exploration
  - **Speeds:** `max_linear_speed_vmax: 0.25`, `max_angular_speed_wmax: 1.0` (match Nav2).
  - **Distances:**
    - `frontier_selection_min_distance: 0.5`, `frontier_candidate_min_goal_distance_m: 0.5`
      (≈2.5 × robot radius 0.18)
    - `frontier_visit_tolerance: 0.4`
    - `sensor_effective_range_m: 1.5`
  - **Lidar model:** `goal_preemption_lidar_fov_deg: 360.0`,
    `goal_preemption_lidar_range_m: 8.0`, `goal_preemption_lidar_yaw_offset_deg: 0.0`.
  - **Robustness:** `frontier_suppression_enabled: true`, `post_goal_settle_enabled: true`
    (`post_goal_min_settle: 1.0`), because the real robot uses Wi-Fi TF and slow SLAM updates.
  - **Map / solver:**
    - `occ_threshold: 65`, `min_frontier_size_cells: 5`
    - `mrtsp_solver: dp`; switch to `greedy` if laptop CPU is too high
    - `map_processing_rate_hz: 1.0`
  - If `/map` comes up empty at startup: `map_qos_autodetect_on_startup: true`.
- [ ] Confirm the exact control-service name/type in the library source (used by the supervisor).
- [ ] `launch/exploration.launch.py` (one command for the whole laptop side):
  - scan filter + SLAM + Nav2 + explorer + supervisor + RViz
  - launch args `max_duration_s`, `stagnation_window_s`, `map_name`, `use_rviz`
- [ ] Test in a closed, small area (doors shut) with someone standing by.

**Done when:** Raupy explores the area on its own and the library publishes
`exploration_complete`.

### Phase 4 – `exploration_supervisor` (our code, ≈1 day, by 2026-09-21)
One well-commented node in `raupy_exploration`.

**Inputs:**
- `/map` (transient local)
- `exploration_complete` (std_msgs/Empty)
- the Nav2 `navigate_to_pose` action status (to cancel goals)

**Stop conditions** (first one wins; parameters in brackets):
1. **Complete:** `exploration_complete` received ("no more frontiers").
2. **No new information:** the number of known cells (free + occupied) in `/map` grows by less
   than `min_new_area_m2` within `stagnation_window_s` [default 0.5 m² in 90 s]. Only
   armed after `startup_grace_s` [20 s].
3. **Time budget:** `max_duration_s` [default 600 s] since start.

**Stop sequence:**
1. Pause the explorer via its control service.
2. Cancel any active `navigate_to_pose` goals.
3. Publish zero `TwistStamped` on `/cmd_vel` for ~1 s.
4. Save the map:
   - `/map_saver/save_map` (nav2_map_server, launched in our launch file) →
     `maps/<map_name>_<timestamp>.pgm/.yaml`
   - `/slam_toolbox/serialize_map` → `.posegraph/.data`
5. Log a summary: reason, duration, explored area in m², number of frontier goals reached if
   available. Also write it to `maps/<…>_summary.yaml`.
6. Shut down (or stay idle, depending on a parameter).

**Tests:**
- trigger each condition on purpose: short `max_duration_s`; robot boxed in for stagnation;
  small closed room for complete
- check that the map files open

### Phase 5 – Real-robot runs & evaluation (≈1.5 days, 2026-09-22 – 09-23)
- [ ] Before each session:
  - charge the battery (lidar + driving drain it)
  - check the domain ID
  - `rosbot-lidar.sh start`
  - `systemctl is-active microros rosbot` on the robot
- [ ] Safety: speeds as above, one person with keyboard teleop / joystick ready, and never
      leave the robot unattended. Losing the laptop link stops it within 0.5 s.
- [ ] Runs:
  1. single room
  2. corridor with several open doors
  3. larger area with the time-budget stop

  Save the maps and summaries from each run.
- [ ] Record one rosbag per showcase run: `/scan /tf /tf_static /odometry/filtered /map
      /explore/frontiers /explore/selected_frontier`, all small.
- [ ] Screen-record RViz for the presentation.
- [ ] Tune based on the runs: inflation / speed if the robot gets stuck, frontier min size if
      it dithers, stagnation window if it stops too early or late.

### Phase 6 – Documentation & presentation (≈1 day, 2026-09-24)
- [ ] `README.md` in the repo root:
  - what it does
  - architecture diagram
  - setup (apt, submodule, build)
  - how to run on Raupy (step by step)
  - parameters, known issues
  - **"Wer hat was gemacht"**
- [ ] All our code and config commented. Commits carry the individual authors.
- [ ] 15-minute slides:
  - task
  - how the lecture's SPLAM idea maps to the library
  - architecture on robot + laptop
  - supervisor stop logic
  - demo video / maps
  - problems and fixes (stamped cmd_vel, no base_footprint, broken `/scan_filtered`, domain ID)
- [ ] Buffer: 2026-09-25 morning.

## 6. Suggested split (4 people)

| Area | Phases |
|---|---|
| A – Repo, environment, SLAM + scan filter | 0, 1 |
| B – Nav2 configuration & tuning | 2 |
| C – Explorer integration, launch files, RViz | 3 |
| D – Supervisor node | 4 |
| All | 5 (robot runs), 6 (README section per person, slides) |

## 7. Risks

| Risk | Mitigation |
|---|---|
| Domain ID changes (new DHCP lease) → laptop sees nothing | `raupy_env.sh` reads it live from the robot every time |
| Wi-Fi latency or dropouts break TF / control | Generous `transform_tolerance`, low speeds, robot stops itself after 0.5 s. Fallback: SLAM on the robot (`rosbot-slam.sh`) |
| Laptop CPU/RAM (4 cores, 3.8 GB) | `mrtsp_solver: greedy`, `map_processing_rate_hz: 0.5`, RViz on a second machine if needed |
| Skid-steer odometry slips when turning | EKF uses IMU yaw; low `max_vel_theta`; `rotate_to_heading` at low speed |
| Low battery during demo | Charge before sessions; lidar only on while testing |
| Laptop/robot clocks out of sync → collision_monitor stops the robot ("invalid source", scan older than 2 s), TF extrapolation errors | Sync both clocks via NTP/chrony before a session (`timedatectl` on both); check `ros2 topic delay /scan` |
| Limited robot time | Prepare configs and launch files before robot slots; test launch files with a recorded rosbag (`--clock`, `use_sim_time`) |
