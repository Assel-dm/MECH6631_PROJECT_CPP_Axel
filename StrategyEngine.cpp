#include "StrategyEngine.h"
#include "ObstaclePipeline.h"
#include <cmath>
#include <iostream>

StrategyEngine::StrategyEngine()
    : offense_(&planner_, &follower_, &fuzzy_),
    defense_(&planner_, &follower_, &fuzzy_)
{
    my_id_ = 0;
    offense_mode_ = true;

    // Grid and obstacle detection parameters
    cell_px_    = 20;
    inflate_px_ = 10;

    // Lab floor colour model thresholds
    kL_ = 2.5f;
    ka_ = 2.0f;
    kb_ = 2.0f;

    // Strategy parameters
    lookahead_cells_  = 4;
    v_max_            = 120.0;
    laser_close_px_   = 140.0;
    laser_align_deg_  = 10.0;

    // Tracker parameters
    max_match_dist_px_ = 60.0;
    max_misses_        = 8;

    // ── Arena boundary parameters ─────────────────────────────────────────────
    // Virtual wall strip width added to the occupancy grid on all four sides.
    // Start at 60px and increase if the robot still approaches the camera edge.
    arena_margin_px_ = 60;

    // Hard-stop zone: halt all motion if tracked centroid enters this border.
    // Keep >= arena_margin_px_ so it only triggers when the path guard fails.
    arena_danger_px_ = 80;
}

// ---------------------------------------------------------------------------
// Appends four axis-aligned Obstacle strips along the camera frame boundary.
// Each strip is arena_margin_px_ thick and spans the full frame width/height.
// ---------------------------------------------------------------------------
void StrategyEngine::addArenaBoundaries(std::vector<Obstacle>& obs, int W, int H) const
{
    int m = arena_margin_px_;

    // Helper: push one rectangle expressed as (top-left x, top-left y, w, h)
    auto push = [&](int x, int y, int w, int h) {
        Obstacle o;
        o.x    = x;
        o.y    = y;
        o.w    = w;
        o.h    = h;
        o.cx   = x + w / 2.0;
        o.cy   = y + h / 2.0;
        o.area = static_cast<double>(w * h);
        obs.push_back(o);
    };

    push(0,     0,     W, m);      // top
    push(0,     H - m, W, m);      // bottom
    push(0,     0,     m, H);      // left
    push(W - m, 0,     m, H);      // right
}

void StrategyEngine::setID(int id) {
    my_id_ = id;
}

Command StrategyEngine::update(image& rgb, double now)
{
    int W = rgb.width;
    int H = rgb.height;

    // ------------------------------------------------------------
    // 1. Detect front (blue) and rear (red) colour markers
    // ------------------------------------------------------------
    std::vector<Blob> front_blobs;
    std::vector<Blob> rear_blobs;

    markerDetector_.detect_markers(rgb, front_blobs, rear_blobs);

    std::vector<Blob> robot_blobs = front_blobs;
    robot_blobs.insert(robot_blobs.end(), rear_blobs.begin(), rear_blobs.end());

    // ------------------------------------------------------------
    // 2. Pair front/rear blobs into single robot detections
    // ------------------------------------------------------------
    std::optional<double> expected_sep;
    auto dets = tracker_.pairMarkers(
        front_blobs,
        rear_blobs,
        expected_sep,
        0.55,
        1200.0
    );

    // ------------------------------------------------------------
    // 3. Update persistent robot tracks
    // ------------------------------------------------------------
    tracks_ = tracker_.updateTracks(
        tracks_,
        dets,
        now,
        max_match_dist_px_,
        max_misses_
    );

    if (tracks_.size() < 2) {
        return { 0.0, 0.0, false };
    }

    // ------------------------------------------------------------
    // 4. Compute robot mask sizes from marker separation
    // ------------------------------------------------------------
    double marker_sep_in   = 9.0;
    double robot_length_in = 15.0;
    double robot_width_in  = 11.0;

    int robot_length_px = 100;
    int robot_width_px  = 80;

    if (!dets.empty()) {
        double avg_sep_px = 0.0;
        for (const auto& d : dets) avg_sep_px += d.sep_px;
        avg_sep_px /= dets.size();

        double px_per_in    = avg_sep_px / marker_sep_in;
        double safety_factor = 3.5;
        robot_length_px = (int)(robot_length_in * px_per_in * safety_factor);
        robot_width_px  = (int)(robot_width_in  * px_per_in * safety_factor);
    }

    // ------------------------------------------------------------
    // 5. Detect real obstacles and build initial occupancy grid
    // ------------------------------------------------------------
    auto pipeline_res = process_frame_obstacles(
        rgb,
        dets,
        obstacleDetector_,
        occBuilder_,
        kL_, ka_, kb_,
        500,
        cell_px_,
        inflate_px_,
        robot_length_px,
        robot_width_px
    );

    auto obstacles = pipeline_res.obstacles;

    // ------------------------------------------------------------
    // 5b. Append virtual arena boundary obstacles and rebuild grids.
    //     This prevents A* from ever planning a path into the border
    //     zone, so the robot stays within the camera's field of view.
    // ------------------------------------------------------------
    addArenaBoundaries(obstacles, W, H);

    Grid grid        = occBuilder_.build(obstacles, W, H, cell_px_, inflate_px_);
    Grid grid_visual = occBuilder_.build(obstacles, W, H, cell_px_, 0);

    // ------------------------------------------------------------
    // 6. Determine offense or defense mode
    // ------------------------------------------------------------
    const RobotTrack* me    = nullptr;
    const RobotTrack* enemy = nullptr;

    for (auto& t : tracks_) {
        if (t.id == my_id_) me = &t;
    }

    if (me) {
        // ── Hard-stop boundary guard ─────────────────────────────────────────
        // If the robot's centroid is already inside the danger zone (e.g. after
        // a shove or tracking lag), stop all motion immediately rather than
        // issuing a command that could push it further out of frame.
        double d = arena_danger_px_;
        bool near_edge = (me->x < d) || (me->x > W - d) ||
                         (me->y < d) || (me->y > H - d);
        if (near_edge) {
            std::cout << "  [BOUNDARY GUARD] robot near edge ("
                      << (int)me->x << "," << (int)me->y
                      << ") — halting." << std::endl;
            return { 0.0, 0.0, false };
        }

        double best_d = 1e9;
        for (auto& t : tracks_) {
            if (t.id == my_id_) continue;
            double dist = std::hypot(t.x - me->x, t.y - me->y);
            if (dist < best_d) {
                best_d = dist;
                enemy  = &t;
            }
        }
        if (enemy) {
            offense_mode_ = (best_d > 120.0);
        }
    }

    // ------------------------------------------------------------
    // 7. Run the active strategy and return the command
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
            grid_visual,
            cell_px_,
            lookahead_cells_,
            80,
            v_max_,
            robot_length_px * 0.5,
            obstacles
        );
        cmd = res.cmd;
    }

    return cmd;
}