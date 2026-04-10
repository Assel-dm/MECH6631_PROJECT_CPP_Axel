#include "Offense.h"
#include <cmath>
#include <algorithm>

static const RobotTrack* find_me(const std::vector<RobotTrack>& tracks, int my_id) {
    for (auto& t : tracks)
        if (t.id == my_id)
            return &t;
    return nullptr;
}

static const RobotTrack* pick_nearest_enemy(const std::vector<RobotTrack>& tracks, int my_id) {
    const RobotTrack* me = find_me(tracks, my_id);
    if (!me) return nullptr;

    const RobotTrack* best = nullptr;
    double best_d = 1e9;

    for (auto& t : tracks) {
        if (t.id == my_id) continue;
        double d = std::hypot(t.x - me->x, t.y - me->y);
        if (d < best_d) {
            best_d = d;
            best = &t;
        }
    }
    return best;
}

OffenseStrategy::OffenseStrategy(AStarPlanner* p,
    WaypointFollower* f,
    FuzzyLogic* fl)
    : planner_(p), follower_(f), fuzzy_(fl) {
}

std::pair<Command, std::optional<std::vector<std::pair<int, int>>>>
OffenseStrategy::compute(
    const std::vector<RobotTrack>& tracks,
    int my_id,
    const Grid& grid,
    int cell_px,
    int lookahead_cells,
    double laser_close_px,
    double laser_align_deg,
    double v_max,
    const std::vector<Obstacle>& obstacles)
{
    Command stop{ 0.0, 0.0, false };

    const RobotTrack* me = find_me(tracks, my_id);
    const RobotTrack* target = pick_nearest_enemy(tracks, my_id);

    if (!me || !target)
        return { stop, std::nullopt };

    // Extract tactical features
    TacticalFeatures feat = fuzzy_->extractFeatures(*me, *target, obstacles);
    FuzzyDecision dec = fuzzy_->offense(feat);

    // Convert to grid coordinates
    int sy = (int)(me->y / cell_px);
    int sx = (int)(me->x / cell_px);
    int gy = (int)(target->y / cell_px);
    int gx = (int)(target->x / cell_px);

    auto path_opt = planner_->plan(grid, { sy, sx }, { gy, gx });
    if (!path_opt || path_opt->size() < 2)
        return { stop, path_opt };

    // Dynamic lookahead
    int dyn_look = std::max(1, (int)std::round(lookahead_cells * dec.lookahead_scale));

    int idx;
    if (dec.prefer_alt_path && (int)path_opt->size() >= 4) {
        idx = std::max(1, std::min((int)path_opt->size() - 2,
            (int)std::round(0.55 * ((int)path_opt->size() - 1))));
    }
    else {
        idx = std::min((int)path_opt->size() - 1, dyn_look);
    }

    auto [wy, wx] = (*path_opt)[idx];
    double wx_pix = (wx + 0.5) * cell_px;
    double wy_pix = (wy + 0.5) * cell_px;

    double v_scaled = v_max * dec.speed_scale;

    Command cmd = follower_->follow(
        me->x, me->y, me->theta,
        { wx_pix, wy_pix },
        20.0,      // stop distance
        2.0,       // angular gain
        0.02,      // linear gain
        v_scaled
    );

    // Laser firing logic
    double dx = target->x - me->x;
    double dy = target->y - me->y;
    double d = std::hypot(dx, dy);
    double desired = std::atan2(dy, dx);
    double err = angle_wrap(desired - me->theta);
    double err_deg = std::fabs(err * 180.0 / M_PI);

    cmd.laser = (d < laser_close_px && err_deg < laser_align_deg);

    return { cmd, path_opt };
}