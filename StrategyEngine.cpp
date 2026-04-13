#include "StrategyEngine.h"
#include "ObstaclePipeline.h"
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

    // Lab floor model thresholds (tune for your lighting)
    kL_ = 2.5f;  // Lightness sensitivity
    ka_ = 2.0f;  // Red-green sensitivity
    kb_ = 2.0f;  // Blue-yellow sensitivity

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
    // 4. Detect obstacles with proper robot body masking
    // ------------------------------------------------------------
    
    // Physical robot dimensions (adjust to your actual robot size)
    double marker_sep_in = 9.0;        // marker separation in inches
    double robot_length_in = 15.0;     // robot length in inches
    double robot_width_in = 11.0;       // robot width in inches
    
    // Compute pixel scale from detected marker separation
    int robot_length_px = 100;  // default fallback
    int robot_width_px = 80;
    
    if (!dets.empty()) {
        double avg_sep_px = 0.0;
        for (const auto& d : dets) avg_sep_px += d.sep_px;
        avg_sep_px /= dets.size();
        
        double px_per_in = avg_sep_px / marker_sep_in;
        
        // ADD safety factor here:
        double safety_factor = 3.5;  // Make masks larger
        robot_length_px = (int)(robot_length_in * px_per_in * safety_factor);
        robot_width_px = (int)(robot_width_in * px_per_in * safety_factor);
    }
    
    // Use pipeline with oriented rectangle masking
    auto pipeline_res = process_frame_obstacles(
        rgb,
        dets,               // Uses RobotDet with pose info
        obstacleDetector_,
        occBuilder_,
        kL_, ka_, kb_,
        500,                // min_area
        cell_px_,
        inflate_px_,
        robot_length_px,
        robot_width_px
    );
    
    auto obstacles = pipeline_res.obstacles;
    auto grid = pipeline_res.occ_grid;

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