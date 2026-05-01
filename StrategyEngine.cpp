#include "StrategyEngine.h"
#include "ObstaclePipeline.h"

#include <cmath>
#include <iostream>
#include <limits>

using namespace ColorProfiles;

StrategyEngine::StrategyEngine()
    : offense_(&planner_, &follower_, &fuzzy_),
      defense_(&planner_, &follower_, &fuzzy_),
      laser_gate_(3, 8)
{
    my_id_ = 0;
    offense_mode_ = true;

    color_specs_ = buildDefaultColorSpecs();
    robot_profiles_ = buildDefaultRobotProfiles();
    for (const auto& kv : robot_profiles_) expected_sep_px_map_[kv.first] = std::nullopt;

    cell_px_ = 20;
    inflate_px_ = 10;

    blob_min_area_ = 60;
    blob_max_area_ = 3000;
    min_area_ratio_ = 0.20;
    pair_sep_tol_ = 0.55;
    max_pair_px_ = 1200.0;
    pair_area_ratio_tol_ = 2.6;
    morph_open_iters_ = 1;
    morph_close_iters_ = 2;
    morph_repeat_ = 1;

    kL_ = 2.5f;
    ka_ = 2.0f;
    kb_ = 2.0f;

    lookahead_cells_ = 4;
    v_max_ = 120.0;  //test : 0.25 reel: 120 ?
    laser_close_px_ = 140.0;
    laser_align_deg_ = 10.0;
    laser_los_margin_px_ = 12.0;

    max_match_dist_px_ = 60.0;
    max_misses_ = 8;

    arena_margin_px_ = 60;
    arena_danger_px_ = 80;
}

std::vector<RobotDet> StrategyEngine::toLegacyDets(const std::vector<ProfiledRobotDet>& dets)
{
    std::vector<RobotDet> out;
    out.reserve(dets.size());
    for (const auto& d : dets) out.push_back(RobotDet{d.front, d.rear, d.x, d.y, d.theta, d.sep_px});
    return out;
}

std::vector<RobotTrack> StrategyEngine::toLegacyTracks(const std::vector<ProfiledRobotTrack>& tracks)
{
    std::vector<RobotTrack> out;
    out.reserve(tracks.size());
    for (const auto& t : tracks) out.push_back(RobotTrack{t.id, t.x, t.y, t.theta, t.sep_px, t.last_seen, t.misses});
    return out;
}

void StrategyEngine::addArenaBoundaries(std::vector<Obstacle>& obs, int W, int H) const
{
    int m = arena_margin_px_;
    auto push = [&](int x, int y, int w, int h) {
        Obstacle o;
        o.x = x; o.y = y; o.w = w; o.h = h;
        o.cx = x + w / 2.0;
        o.cy = y + h / 2.0;
        o.area = static_cast<double>(w * h);
        obs.push_back(o);
    };
    push(0, 0, W, m);
    push(0, H - m, W, m);
    push(0, 0, m, H);
    push(W - m, 0, m, H);
}

void StrategyEngine::setID(int id)
{
    my_id_ = id;
}

const StrategyDebugInfo& StrategyEngine::debugInfo() const
{
    return last_debug_;
}

Command StrategyEngine::update(image& rgb, double now)
{
    const int W = rgb.width;
    const int H = rgb.height;

    last_debug_ = StrategyDebugInfo{};
    last_debug_.my_id = my_id_;
    last_debug_.offense_mode = offense_mode_;

    std::map<std::string, std::optional<double>> sep_estimates;
    std::vector<ProfiledRobotDet> prof_dets = detectProfileRobots(
        rgb, color_specs_, robot_profiles_, expected_sep_px_map_,
        blob_min_area_, blob_max_area_, min_area_ratio_, pair_sep_tol_, max_pair_px_,
        pair_area_ratio_tol_, morph_open_iters_, morph_close_iters_, morph_repeat_,
        nullptr, nullptr, &sep_estimates);

    for (const auto& kv : sep_estimates) {
        if (!expected_sep_px_map_[kv.first].has_value() && kv.second.has_value()) {
            expected_sep_px_map_[kv.first] = kv.second;
        }
    }

    prof_tracks_ = updateProfileTracks(prof_tracks_, prof_dets, now, max_match_dist_px_, max_misses_);
    tracks_ = toLegacyTracks(prof_tracks_);
    last_debug_.num_tracks = static_cast<int>(tracks_.size());

    if (tracks_.size() < 2) return {0.0, 0.0, false};

    const double marker_sep_in = 9.0;
    const double robot_length_in = 15.0;
    const double robot_width_in = 11.0;

    int robot_length_px = 100;
    int robot_width_px = 80;

    if (!prof_dets.empty()) {
        double avg_sep_px = 0.0;
        for (const auto& d : prof_dets) avg_sep_px += d.sep_px;
        avg_sep_px /= static_cast<double>(prof_dets.size());
        const double px_per_in = avg_sep_px / marker_sep_in;
        const double safety_factor = 3.5;
        robot_length_px = static_cast<int>(robot_length_in * px_per_in * safety_factor);
        robot_width_px = static_cast<int>(robot_width_in * px_per_in * safety_factor);
    }

    std::vector<RobotDet> legacy_dets = toLegacyDets(prof_dets);
    auto pipeline_res = process_frame_obstacles(
        rgb, legacy_dets, obstacleDetector_, occBuilder_,
        kL_, ka_, kb_, 500, cell_px_, inflate_px_, robot_length_px, robot_width_px);

    auto obstacles = pipeline_res.obstacles;
    addArenaBoundaries(obstacles, W, H);
    last_debug_.num_obstacles = static_cast<int>(obstacles.size());

    Grid grid = occBuilder_.build(obstacles, W, H, cell_px_, inflate_px_);
    Grid grid_visual = occBuilder_.build(obstacles, W, H, cell_px_, 0);

    const RobotTrack* me = nullptr;
    const RobotTrack* enemy = nullptr;

    for (const auto& t : tracks_) {
        if (t.id == my_id_) { me = &t; break; }
    }

    if (me) {
        const double d = arena_danger_px_;
        const bool near_edge = (me->x < d) || (me->x > W - d) || (me->y < d) || (me->y > H - d);
        if (near_edge) {
            std::cout << "  [BOUNDARY GUARD] robot near edge ("
                      << static_cast<int>(me->x) << "," << static_cast<int>(me->y)
                      << ") - halting." << std::endl;
            last_debug_.boundary_guard = true;
            last_debug_.my_x = me->x;
            last_debug_.my_y = me->y;
            last_debug_.my_theta = me->theta;
            return {0.0, 0.0, false};
        }

        double best_d = 1e9;
        for (const auto& t : tracks_) {
            if (t.id == my_id_) continue;
            const double dxy = std::hypot(t.x - me->x, t.y - me->y);
            if (dxy < best_d) { best_d = dxy; enemy = &t; }
        }
        if (enemy) {
            offense_mode_ = (best_d > 120.0);
            last_debug_.enemy_id = enemy->id;
            last_debug_.enemy_x = enemy->x;
            last_debug_.enemy_y = enemy->y;
            last_debug_.enemy_dist_px = best_d;
            last_debug_.enemy_bearing_deg = std::atan2(enemy->y - me->y, enemy->x - me->x) * 180.0 / M_PI;
        }

        last_debug_.my_x = me->x;
        last_debug_.my_y = me->y;
        last_debug_.my_theta = me->theta;

        double nearest_obs = std::numeric_limits<double>::infinity();
        for (const auto& o : obstacles) {
            const double dd = std::hypot(o.cx - me->x, o.cy - me->y);
            if (dd < nearest_obs) nearest_obs = dd;
        }
        if (std::isfinite(nearest_obs)) last_debug_.nearest_obstacle_dist_px = nearest_obs;
    }

    last_debug_.offense_mode = offense_mode_;

    Command cmd{0.0, 0.0, false};

    if (offense_mode_) {
        auto res = offense_.compute(
            tracks_, my_id_, grid, cell_px_, lookahead_cells_, laser_close_px_,
            laser_align_deg_, laser_los_margin_px_, v_max_, obstacles);
        cmd = res.cmd;
        cmd.laser = laser_gate_.update(res.target_id, res.request_fire);
    }
    else {
        auto res = defense_.compute(
            tracks_, my_id_, grid, grid_visual, cell_px_, lookahead_cells_, 80,
            v_max_, robot_length_px * 0.5, obstacles);
        cmd = res.cmd;
        cmd.laser = false;
        laser_gate_.reset();
    }

    return cmd;
}
