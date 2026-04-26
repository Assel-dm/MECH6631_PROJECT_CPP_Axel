#include "Offense.h"
#include "GridUtils.h"
#include "CombatGeometry.h"

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

OffenseStrategy::OffenseStrategy(AStarPlanner* planner,
                                 WaypointFollower* follower,
                                 FuzzyLogic* fuzzy)
    : planner_(planner), follower_(follower), fuzzy_(fuzzy)
{}

OffenseStrategy::Result OffenseStrategy::compute(
    const std::vector<RobotTrack>& tracks,
    int my_id,
    const Grid& grid,
    int cell_px,
    int lookahead_cells,
    double laser_close_px,
    double laser_align_deg,
    double laser_los_margin_px,
    double v_max,
    const std::vector<Obstacle>& obstacles)
{
    Result res{};
    res.cmd = {0.0, 0.0, false};
    res.path = std::nullopt;
    res.target_id = std::nullopt;
    res.request_fire = false;

    const RobotTrack* me = find_me(tracks, my_id);
    const RobotTrack* enemy = pick_nearest_enemy(tracks, my_id);
    if (!me || !enemy) return res;

    TacticalFeatures feat = fuzzy_->extractFeatures(*me, *enemy, obstacles);
    FuzzyDecision dec = fuzzy_->offense(feat);

    auto start = GridUtils::safePixToFreeCell(grid, me->x, me->y, cell_px);
    auto goal  = GridUtils::safePixToFreeCell(grid, enemy->x, enemy->y, cell_px);
    if (!start.has_value() || !goal.has_value()) {
        return res;
    }

    auto path = planner_->plan(grid, start.value(), goal.value());
    if (!path.has_value() || path->size() < 2) {
        return res;
    }
    res.path = path;

    int dyn_look = std::max(1, static_cast<int>(std::round(lookahead_cells * dec.lookahead_scale)));
    int idx = 1;
    if (dec.prefer_alt_path && path->size() >= 4) {
        idx = std::max(1, std::min(static_cast<int>(path->size()) - 2,
                                   static_cast<int>(std::round(0.55 * (path->size() - 1)))));
    } else {
        idx = std::min(static_cast<int>(path->size()) - 1, dyn_look);
    }

    const auto [row, col] = (*path)[idx];
    auto waypoint = GridUtils::cellToPix(row, col, cell_px);

    const double v_scaled = v_max * dec.speed_scale;
    res.cmd = follower_->follow(me->x, me->y, me->theta,
                                waypoint,
                                5.0,
                                2.0,
                                0.3,
                                v_scaled);
    res.cmd.laser = false;

    const double dx = enemy->x - me->x;
    const double dy = enemy->y - me->y;
    const double dist_to_enemy = std::hypot(dx, dy);
    const double desired = std::atan2(dy, dx);
    const double err = angle_wrap(desired - me->theta);

    const bool los_ok = CombatGeometry::lineOfSightClear(me->x, me->y,
                                                         enemy->x, enemy->y,
                                                         obstacles,
                                                         laser_los_margin_px);
    const bool close_ok = dist_to_enemy < laser_close_px;
    const bool align_ok = std::fabs(err * 180.0 / M_PI) < laser_align_deg;
    const bool target_ok = (enemy->misses == 0);

    res.target_id = enemy->id;
    res.request_fire = (los_ok && close_ok && align_ok && target_ok);
    return res;
}