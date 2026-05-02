#include "StrategyEngine.h"
#include "ObstaclePipeline.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace ColorProfiles;

StrategyEngine::StrategyEngine()
    : offense_(&planner_, &follower_, &fuzzy_),
      defense_(&planner_, &follower_, &fuzzy_),
      laser_gate_(3, 8)
{
    my_id_ = -1;
    offense_mode_ = true;

    forced_mode_ = PROGRAM_MODE;
    known_profile_ = PROGRAM_MY_PROFILE;
    use_id_dance_ = (PROGRAM_USE_ID_DANCE != 0);

    // Build default multi-profile color logic
    color_specs_ = buildDefaultColorSpecs();
    robot_profiles_ = buildDefaultRobotProfiles();
    for (const auto& kv : robot_profiles_) {
        expected_sep_px_map_[kv.first] = std::nullopt;
    }

    // Internal ID dance configuration
    dance_ = {
        {0.30, +0.45, -0.45, false, +1},
        {0.30, -0.45, +0.45, false, -1},
        {0.30, +0.45, -0.45, false, +1},
        {0.30,  0.00,  0.00, false,  0}
    };
    dance_active_ = false;
    dance_segment_idx_ = 0;
    dance_segment_t0_ = 0.0;
    dance_attempts_left_ = 2;
    id_min_energy_ = 1.5;

    // Grid and obstacle detection parameters
    cell_px_    = 20;
    inflate_px_ = 10;

    // Marker / profile detection parameters
    blob_min_area_ = 60;
    blob_max_area_ = 3000;
    min_area_ratio_ = 0.20;
    pair_sep_tol_ = 0.55;
    max_pair_px_ = 1200.0;
    pair_area_ratio_tol_ = 2.6;
    morph_open_iters_ = 1;
    morph_close_iters_ = 2;
    morph_repeat_ = 1;

    // Lab floor colour model thresholds
    kL_ = 2.5f;
    ka_ = 2.0f;
    kb_ = 2.0f;

    // Strategy parameters
    lookahead_cells_   = 4;
    v_max_             = 120.0;
    laser_close_px_    = 140.0;
    laser_align_deg_   = 10.0;
    laser_los_margin_px_ = 12.0;

    // Tracker parameters
    max_match_dist_px_ = 60.0;
    max_misses_        = 8;

    // Arena boundary parameters
    arena_margin_px_ = 60;
    arena_danger_px_ = 80;
}

void StrategyEngine::setID(int id)
{
    my_id_ = id;
}

void StrategyEngine::setForcedMode(int mode)
{
    forced_mode_ = mode;
}

void StrategyEngine::setKnownProfile(int profile)
{
    known_profile_ = profile;
}

void StrategyEngine::setUseIDDance(bool enabled)
{
    use_id_dance_ = enabled;
}

std::vector<RobotDet> StrategyEngine::toLegacyDets(const std::vector<ProfiledRobotDet>& dets)
{
    std::vector<RobotDet> out;
    out.reserve(dets.size());
    for (const auto& d : dets) {
        out.push_back(RobotDet{d.front, d.rear, d.x, d.y, d.theta, d.sep_px});
    }
    return out;
}

std::vector<RobotTrack> StrategyEngine::toLegacyTracks(const std::vector<ProfiledRobotTrack>& tracks)
{
    std::vector<RobotTrack> out;
    out.reserve(tracks.size());
    for (const auto& t : tracks) {
        out.push_back(RobotTrack{t.id, t.x, t.y, t.theta, t.sep_px, t.last_seen, t.misses});
    }
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

    push(0,     0,     W, m);
    push(0,     H - m, W, m);
    push(0,     0,     m, H);
    push(W - m, 0,     m, H);
}

bool StrategyEngine::selectKnownProfileID()
{
    if (known_profile_ == PROFILE_AUTO) return false;

    std::string wanted_profile;
    if (known_profile_ == PROFILE_GR) wanted_profile = "GR";
    else if (known_profile_ == PROFILE_OB) wanted_profile = "OB";
    else return false;

    int best_id = -1;
    int best_hits = -1;

    for (const auto& t : prof_tracks_) {
        if (t.profile_name == wanted_profile && t.misses == 0) {
            if (t.stable_hits > best_hits) {
                best_hits = t.stable_hits;
                best_id = t.id;
            }
        }
    }

    if (best_id >= 0) {
        my_id_ = best_id;
        return true;
    }
    return false;
}

void StrategyEngine::startDance(double now)
{
    dance_active_ = true;
    dance_segment_idx_ = 0;
    dance_segment_t0_ = now;
    id_prev_theta_.clear();
    id_prev_time_.clear();
    id_score_.clear();
    id_energy_.clear();
}

bool StrategyEngine::advanceDanceIfNeeded(double now)
{
    if (!dance_active_) return false;
    if (dance_segment_idx_ < 0 || dance_segment_idx_ >= static_cast<int>(dance_.size())) return false;

    const DanceSegment& seg = dance_[dance_segment_idx_];
    if (now - dance_segment_t0_ >= seg.duration_s) {
        ++dance_segment_idx_;
        dance_segment_t0_ = now;
        if (dance_segment_idx_ >= static_cast<int>(dance_.size())) {
            dance_active_ = false;
            return false;
        }
    }
    return dance_active_;
}

void StrategyEngine::updateDanceScores(double now, int expected_sign)
{
    const double omega_clip = 8.0;

    for (const auto& tr : prof_tracks_) {
        const int tid = tr.id;
        if (id_prev_theta_.find(tid) == id_prev_theta_.end()) {
            id_prev_theta_[tid] = tr.theta;
            id_prev_time_[tid] = now;
            id_score_[tid] = 0.0;
            id_energy_[tid] = 0.0;
            continue;
        }

        const double dt = now - id_prev_time_[tid];
        if (dt <= 1e-6) continue;

        const double dth = angle_wrap(tr.theta - id_prev_theta_[tid]);
        const double omega = clamp(dth / dt, -omega_clip, omega_clip);

        if (expected_sign == 0) {
            id_score_[tid] += -std::fabs(omega);
        }
        else {
            id_score_[tid] += expected_sign * omega;
            id_energy_[tid] += std::fabs(omega);
        }

        id_prev_theta_[tid] = tr.theta;
        id_prev_time_[tid] = now;
    }
}

int StrategyEngine::pickBestDanceID() const
{
    int best_id = -1;
    double best_score = -1e18;

    // First try only energetic candidates.
    for (const auto& kv : id_score_) {
        const int tid = kv.first;
        const double sc = kv.second;
        auto itE = id_energy_.find(tid);
        const double energy = (itE == id_energy_.end()) ? 0.0 : itE->second;
        if (energy >= id_min_energy_ && sc > best_score) {
            best_score = sc;
            best_id = tid;
        }
    }

    // Fallback if the dance was weak but still produced scores.
    if (best_id < 0) {
        for (const auto& kv : id_score_) {
            if (kv.second > best_score) {
                best_score = kv.second;
                best_id = kv.first;
            }
        }
    }

    return best_id;
}

Command StrategyEngine::runIdentificationDance(double now)
{
    if (my_id_ >= 0) return {0.0, 0.0, false};

    if (prof_tracks_.empty()) {
        return {0.0, 0.0, false};
    }

    if (!dance_active_) {
        if (dance_attempts_left_ <= 0) {
            const int picked = pickBestDanceID();
            if (picked >= 0) {
                my_id_ = picked;
                std::cout << "[ID DANCE] selected robot id=" << my_id_ << std::endl;
            }
            return {0.0, 0.0, false};
        }
        --dance_attempts_left_;
        startDance(now);
    }

    if (!advanceDanceIfNeeded(now)) {
        const int picked = pickBestDanceID();
        if (picked >= 0) {
            my_id_ = picked;
            std::cout << "[ID DANCE] selected robot id=" << my_id_ << std::endl;
        }
        return {0.0, 0.0, false};
    }

    const DanceSegment& seg = dance_[dance_segment_idx_];
    updateDanceScores(now, seg.expected_omega_sign);
    return {seg.left, seg.right, seg.laser};
}

Command StrategyEngine::update(image& rgb, double now)
{
    const int W = rgb.width;
    const int H = rgb.height;

    // ---------------------------------------------------------------------
    // 1) Multi-profile detection (GR / OB) using explicit color specs
    // ---------------------------------------------------------------------
    std::map<std::string, std::optional<double>> sep_estimates;
    std::vector<ProfiledRobotDet> prof_dets = detectProfileRobots(
        rgb,
        color_specs_,
        robot_profiles_,
        expected_sep_px_map_,
        blob_min_area_,
        blob_max_area_,
        min_area_ratio_,
        pair_sep_tol_,
        max_pair_px_,
        pair_area_ratio_tol_,
        morph_open_iters_,
        morph_close_iters_,
        morph_repeat_,
        nullptr,
        nullptr,
        &sep_estimates);

    for (const auto& kv : sep_estimates) {
        if (!expected_sep_px_map_[kv.first].has_value() && kv.second.has_value()) {
            expected_sep_px_map_[kv.first] = kv.second;
        }
    }

    // ---------------------------------------------------------------------
    // 2) Profile-aware tracking, then convert to legacy RobotTrack so the
    //    existing fuzzy/offense/defense code can continue to run unchanged.
    // ---------------------------------------------------------------------
    prof_tracks_ = updateProfileTracks(
        prof_tracks_,
        prof_dets,
        now,
        max_match_dist_px_,
        max_misses_);
    tracks_ = toLegacyTracks(prof_tracks_);

    // Known-profile compulsory folders: no ID dance, select our own robot by profile.
    if (known_profile_ != PROFILE_AUTO) {
        selectKnownProfileID();
    }
    // Challenge folders: profile unknown, identify by motion signature.
    else if (use_id_dance_ && my_id_ < 0) {
        return runIdentificationDance(now);
    }

    if (my_id_ < 0) {
        return {0.0, 0.0, false};
    }

    if (tracks_.size() < 2) {
        return {0.0, 0.0, false};
    }

    // ---------------------------------------------------------------------
    // 3) Robot mask sizes from marker separation
    // ---------------------------------------------------------------------
    const double marker_sep_in   = 9.0;
    const double robot_length_in = 15.0;
    const double robot_width_in  = 11.0;

    int robot_length_px = 100;
    int robot_width_px  = 80;

    if (!prof_dets.empty()) {
        double avg_sep_px = 0.0;
        for (const auto& d : prof_dets) avg_sep_px += d.sep_px;
        avg_sep_px /= static_cast<double>(prof_dets.size());

        const double px_per_in = avg_sep_px / marker_sep_in;
        const double safety_factor = 3.5;
        robot_length_px = static_cast<int>(robot_length_in * px_per_in * safety_factor);
        robot_width_px  = static_cast<int>(robot_width_in  * px_per_in * safety_factor);
    }

    // ---------------------------------------------------------------------
    // 4) Detect obstacles and build occupancy grid, excluding robot bodies
    // ---------------------------------------------------------------------
    std::vector<RobotDet> legacy_dets = toLegacyDets(prof_dets);
    auto pipeline_res = process_frame_obstacles(
        rgb,
        legacy_dets,
        obstacleDetector_,
        occBuilder_,
        kL_, ka_, kb_,
        500,
        cell_px_,
        inflate_px_,
        robot_length_px,
        robot_width_px);

    auto obstacles = pipeline_res.obstacles;
    addArenaBoundaries(obstacles, W, H);

    Grid grid        = occBuilder_.build(obstacles, W, H, cell_px_, inflate_px_);
    Grid grid_visual = occBuilder_.build(obstacles, W, H, cell_px_, 0);

    // ---------------------------------------------------------------------
    // 5) Determine offense / defense mode from ProgramVariant.h
    // ---------------------------------------------------------------------
    const RobotTrack* me = nullptr;
    for (const auto& t : tracks_) {
        if (t.id == my_id_) {
            me = &t;
            break;
        }
    }

    if (me) {
        const double d = arena_danger_px_;
        const bool near_edge = (me->x < d) || (me->x > W - d) ||
                               (me->y < d) || (me->y > H - d);
        if (near_edge) {
            std::cout << "  [BOUNDARY GUARD] robot near edge ("
                      << static_cast<int>(me->x) << "," << static_cast<int>(me->y)
                      << ") — halting." << std::endl;
            return {0.0, 0.0, false};
        }
    }

    if (forced_mode_ == MODE_OFFENSE) {
        offense_mode_ = true;
    }
    else if (forced_mode_ == MODE_DEFENSE) {
        offense_mode_ = false;
    }
    else {
        // Optional auto mode: preserve previous behavior.
        const RobotTrack* enemy = nullptr;
        if (me) {
            double best_d = 1e9;
            for (const auto& t : tracks_) {
                if (t.id == my_id_) continue;
                const double dxy = std::hypot(t.x - me->x, t.y - me->y);
                if (dxy < best_d) {
                    best_d = dxy;
                    enemy = &t;
                }
            }
            if (enemy) offense_mode_ = (best_d > 120.0);
        }
    }

    // ---------------------------------------------------------------------
    // 6) Run strategy. Offense returns a fire request + target id and the
    //    final laser command is filtered through LaserGate.
    // ---------------------------------------------------------------------
    Command cmd{0.0, 0.0, false};

    if (offense_mode_) {
        auto res = offense_.compute(
            tracks_,
            my_id_,
            grid,
            cell_px_,
            lookahead_cells_,
            laser_close_px_,
            laser_align_deg_,
            laser_los_margin_px_,
            v_max_,
            obstacles);

        cmd = res.cmd;
        cmd.laser = laser_gate_.update(res.target_id, res.request_fire);
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
            obstacles);
        cmd = res.cmd;
        cmd.laser = false;
        laser_gate_.reset();
    }

    return cmd;
}
