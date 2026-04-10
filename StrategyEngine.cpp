#include "StrategyEngine.h"
#include <cmath>
#include <iostream>

StrategyEngine::StrategyEngine()
    : offense_(&planner_, &follower_, &fuzzy_),
    defense_(&planner_, &follower_, &fuzzy_)
{
    my_id_ = 0;              // You can update this after ID dance
    offense_mode_ = true;    // Start in offense mode

    // Grid + obstacle parameters
    cell_px_ = 20;
    inflate_px_ = 10;

    // Strategy parameters
    lookahead_cells_ = 4;
    v_max_ = 120.0;
    laser_close_px_ = 140.0;
    laser_align_deg_ = 10.0;

    // Tracking parameters
    max_match_dist_px_ = 60.0;
    max_misses_ = 8;
}
void StrategyEngine::setID(int id) {
    my_id_ = id;
}
Command StrategyEngine::update(image& rgb, double now)
{
    int W = rgb.width;
    int H = rgb.height;

    // ------------------------------------------------------------
    // 1. Detect front (blue) and rear (red) markers
    // ------------------------------------------------------------
    std::vector<Blob> front_blobs;
    std::vector<Blob> rear_blobs;

    markerDetector_.detect_markers(rgb, front_blobs, rear_blobs);

    // combine front + rear blobs into robot_blobs (insert after detect_markers)
    std::vector<Blob> robot_blobs = front_blobs;
    robot_blobs.insert(robot_blobs.end(), rear_blobs.begin(), rear_blobs.end());

    // ------------------------------------------------------------
    // 2. Pair markers into robot detections
    // ------------------------------------------------------------
    std::optional<double> expected_sep; // optional EMA if you want
    auto dets = tracker_.pairMarkers(
        front_blobs,
        rear_blobs,
        expected_sep,
        0.55,       // sep tolerance
        1200.0      // max pairing distance
    );

    // ------------------------------------------------------------
    // 3. Update robot tracks
    // ------------------------------------------------------------
    tracks_ = tracker_.updateTracks(
        tracks_,
        dets,
        now,
        max_match_dist_px_,
        max_misses_
    );

    // If we don't have at least 2 robots, we can't strategize
    if (tracks_.size() < 2) {
        return { 0.0, 0.0, false };
    }

    // ------------------------------------------------------------
    // 4. Detect obstacles (HSV histogram on saturation)
    // ------------------------------------------------------------
    auto obstacles = obstacleDetector_.detect(
        rgb,
        robot_blobs,
        markerDetector_.blue_range,
        markerDetector_.red_range,
        500 // choose min_area appropriate for your frame size
    );

    // ------------------------------------------------------------
    // 5. Build occupancy grid
    // ------------------------------------------------------------
    auto grid = occBuilder_.build(
        obstacles,
        W, H,
        cell_px_,
        inflate_px_
    );

    // ------------------------------------------------------------
    // 6. Determine offense or defense mode
    // ------------------------------------------------------------
    const RobotTrack* me = nullptr;
    const RobotTrack* enemy = nullptr;

    for (auto& t : tracks_) {
        if (t.id == my_id_) me = &t;
    }

    if (me) {
        double best_d = 1e9;
        for (auto& t : tracks_) {
            if (t.id == my_id_) continue;
            double d = std::hypot(t.x - me->x, t.y - me->y);
            if (d < best_d) {
                best_d = d;
                enemy = &t;
            }
        }

        if (enemy) {
            // Simple rule: far = offense, close = defense
            offense_mode_ = (best_d > 120.0);
        }
    }

    // ------------------------------------------------------------
    // 7. Compute strategy command
    // ------------------------------------------------------------
    Command cmd{ 0.0, 0.0, false };

    if (offense_mode_) {
        auto [c, path] = offense_.compute(
            tracks_,
            my_id_,
            grid,
            cell_px_,
            lookahead_cells_,
            laser_close_px_,
            laser_align_deg_,
            v_max_,
            obstacles
        );
        cmd = c;
    }
    else {
        auto res = defense_.compute(
            tracks_,
            my_id_,
            grid,
            cell_px_,
            lookahead_cells_,
            /*samples*/ 80,
            v_max_,
            obstacles
        );
        cmd = res.cmd;
    }
    return cmd;
}