# MECH6631 Physical Robot — Teammate Guide

This document explains the structure of the codebase so you can quickly locate
and modify the right file for any physical testing change.

---

## System Architecture

The PC does **all** computation. The Arduino only receives pre-computed motor
pulse-width commands over a Bluetooth serial link and drives the wheels/laser
directly. No strategy logic runs on the Arduino.

---

## File Reference

### Entry Point

| File | Purpose |
|---|---|
| [`program.cpp`](program.cpp) | Main loop. Grab frame → ID dance → strategy → send command. **Start here.** |

### Configuration (change these most often)

All the values below live at the top of `main()` in [`program.cpp`](program.cpp):

| Constant | What it does |
|---|---|
| `PHYS_MODE` | `0` = navigation/detection tuning only, `1` = full strategy |
| `BT_PORT` | Windows COM port for Bluetooth (check Device Manager) |
| `BT_BAUD` | Must match the Arduino sketch baud rate (default `9600`) |
| `CAM_INDEX` | USB camera index — try `1` or `2` if `0` fails |
| `TEST_DURATION` | Auto-stop time in seconds (`0` = run until 'X' is pressed) |

Tunable control and detection parameters live in `StrategyEngine::StrategyEngine()` in [`StrategyEngine.cpp`](StrategyEngine.cpp):

| Parameter | What it does |
|---|---|
| `cell_px_` | Occupancy grid cell size (px). Smaller = finer path, slower. |
| `inflate_px_` | Obstacle inflation radius (px). Increase if robot clips obstacles. |
| `kL_`, `ka_`, `kb_` | Floor colour model thresholds. Tune under your lab lighting. |
| `v_max_` | Maximum normalised wheel speed sent to the Arduino. |
| `laser_close_px_` | Distance (px) at which offense attempts to fire the laser. |
| `laser_align_deg_` | Heading error tolerance (°) for laser fire. |
| `lookahead_cells_` | How far ahead the waypoint follower looks on the path. |
| `max_match_dist_px_` | Maximum pixel distance to match a detection to an existing track. |
| `max_misses_` | Frames a track can go undetected before being dropped. |
| `arena_margin_px_` | Width (px) of virtual wall strips added along all four camera-frame edges. A* will never plan into this border. Increase if the robot still approaches the edge during play. |
| `arena_danger_px_` | Hard-stop threshold (px). If the robot's tracked centroid enters this border, all motion is halted immediately regardless of the planned path. Keep ≥ `arena_margin_px_`. |

---

### Arena Boundary Enforcement

The robot has no physical walls to stop it leaving the camera's field of view.
Two complementary mechanisms in `StrategyEngine::update()` handle this:

1. **Virtual boundary obstacles** — `addArenaBoundaries()` appends four
   `arena_margin_px_`-thick `Obstacle` strips (top / bottom / left / right) to
   the real obstacle list before the occupancy grids are rebuilt each frame.
   Because A* plans on the inflated grid, these strips act as keep-out zones and
   guide both offense and defense paths away from the edges automatically.

2. **Hard-stop guard** — after tracking, if the robot's centroid is within
   `arena_danger_px_` of any frame edge (e.g. after a collision or brief
   tracking loss), `update()` returns `{0, 0, false}` immediately, overriding
   whatever A* would have output.

> **Tuning tip:** run with the default values first. If the robot still gets too
> close to an edge, increase `arena_margin_px_` in steps of 10–20 px.
> Only increase `arena_danger_px_` if collisions are physically pushing the
> robot out of frame before the path guard can react.

---

### Detection & Tracking

| File | Purpose |
|---|---|
| [`MarkerDetector.h`](MarkerDetector.h) / [`MarkerDetector.cpp`](MarkerDetector.cpp) | HSV colour segmentation to find blue (front) and red (rear) marker blobs. Tune `blue_ranges`, `red_ranges`, `min_blob_area`, `max_blob_area` if markers are missed or noisy. |
| [`Tracking.h`](Tracking.h) / [`Tracking.cpp`](Tracking.cpp) | Pairs front/rear blobs into robot poses (`RobotDet`). Maintains `RobotTrack` objects over time with nearest-neighbour matching. |
| [`Obstacles.h`](Obstacles.h) / [`Obstacles.cpp`](Obstacles.cpp) | Extracts obstacle bounding boxes from a binary segmentation mask. |
| [`OccupancyGrid.h`](OccupancyGrid.h) / [`OccupancyGrid.cpp`](OccupancyGrid.cpp) | Converts obstacle list into a 2-D occupancy grid with configurable inflation. |
| [`ObstaclePipeline.h`](ObstaclePipeline.h) / [`ObstaclePipeline.cpp`](ObstaclePipeline.cpp) | Chains `MarkerDetector` masking → `Obstacles` → `OccupancyGrid` into a single `process_frame_obstacles()` call. |

---

### Strategy

| File | Purpose |
|---|---|
| [`StrategyEngine.h`](StrategyEngine.h) / [`StrategyEngine.cpp`](StrategyEngine.cpp) | Top-level controller. Runs the full perception-to-command pipeline each frame. Switches between offense and defense based on distance to nearest enemy. Enforces arena boundary keep-out via virtual obstacles and a hard-stop guard. |
| [`Offense.h`](Offense.h) / [`Offense.cpp`](Offense.cpp) | Drives toward the nearest enemy using A* + waypoint following, fires the laser when close and aligned. |
| [`Defense.h`](Defense.h) / [`Defense.cpp`](Defense.cpp) | Finds the best obstacle to hide behind, plans a path to it, and orbits to maintain cover. |
| [`Fuzzy.h`](Fuzzy.h) / [`Fuzzy.cpp`](Fuzzy.cpp) | Extracts tactical features (distance, alignment, obstacle count) and uses fuzzy membership functions to scale speed and lookahead. |

---

### Navigation

| File | Purpose |
|---|---|
| [`AStar.h`](AStar.h) / [`AStar.cpp`](AStar.cpp) | Grid-based A* path planner. Returns a cell list from current position to goal. |
| [`Waypoint.h`](Waypoint.h) / [`Waypoint.cpp`](Waypoint.cpp) | Proportional controller that generates left/right wheel commands to steer toward a waypoint. |

---

### Robot Identity

| File | Purpose |
|---|---|
| [`IDDance.h`](IDDance.h) / [`IDDance.cpp`](IDDance.cpp) | Spins the robot in place for `spin_time_` seconds. Identifies our robot as the one whose angular velocity best matches the commanded spin rate. Tune `spin_time_` to achieve a clean 360° rotation on your floor surface. |

---

### Shared Types & Utilities

| File | Purpose |
|---|---|
| [`Types.h`](Types.h) | All shared structs (`Blob`, `RobotDet`, `RobotTrack`, `Obstacle`, `Command`, `Grid`) and inline helpers (`vel_to_pw`, `angle_wrap`, `clamp`, `dist2d`). |
| [`ArenaSetup.h`](ArenaSetup.h) / [`ArenaSetup.cpp`](ArenaSetup.cpp) | Obstacle layout presets used by the **simulation** test programs only. Not used in `program.cpp`. |
| [`DebugVisualizer.h`](DebugVisualizer.h) / [`DebugVisualizer.cpp`](DebugVisualizer.cpp) | Draws overlays on the image for simulation debugging. Not used in `program.cpp`. |
| [`timer.h`](timer.h) | `high_resolution_time()` and `pause()` wrappers. |

