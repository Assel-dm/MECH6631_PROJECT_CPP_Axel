#include "Defense.h"
#include "GridUtils.h"
#include "CombatGeometry.h"

#include <cmath>
#include <algorithm>
#include <random>

static const RobotTrack* find_me(const std::vector<RobotTrack>& tracks, int my_id) {
    for (auto& t : tracks)
        if (t.id == my_id) return &t;
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
        if (d < best_d) { best_d = d; best = &t; }
    }
    return best;
}

DefenseStrategy::DefenseStrategy(AStarPlanner* planner,
                                 WaypointFollower* follower,
                                 FuzzyLogic* fuzzy)
    : planner_(planner), follower_(follower), fuzzy_(fuzzy)
{}

DefenseStrategy::Result DefenseStrategy::compute(
    const std::vector<RobotTrack>& tracks,
    int my_id,
    const Grid& grid,
    const Grid& grid_visual,
    int cell_px,
    int lookahead_cells,
    int samples,
    double v_max,
    double robot_half_length_px,
    const std::vector<Obstacle>& obstacles)
{
    Result res{};
    res.cmd = {0.0, 0.0, false};
    res.path = std::nullopt;
    res.hiding_point = std::nullopt;

    const RobotTrack* me = find_me(tracks, my_id);
    const RobotTrack* enemy = pick_nearest_enemy(tracks, my_id);
    if (!me || !enemy) return res;

    TacticalFeatures feat = fuzzy_->extractFeatures(*me, *enemy, obstacles);
    FuzzyDecision dec = fuzzy_->defense(feat);
    res.decision = dec;

    const double v_scaled = v_max * dec.speed_scale;
    const int dyn_look = std::max(1, static_cast<int>(std::round(lookahead_cells * dec.lookahead_scale)));

    const double laser_x = enemy->x + 0.5 * enemy->sep_px * std::cos(enemy->theta);
    const double laser_y = enemy->y + 0.5 * enemy->sep_px * std::sin(enemy->theta);

    const double sep_half = 0.5 * me->sep_px;
    const double red_x_now = me->x - sep_half * std::cos(me->theta);
    const double red_y_now = me->y - sep_half * std::sin(me->theta);

    const bool currently_hidden = !CombatGeometry::lineOfSightClear(laser_x, laser_y,
                                                                    red_x_now, red_y_now,
                                                                    obstacles,
                                                                    10.0);

    std::optional<std::vector<std::pair<int,int>>> path;

    // 1) If already hidden, orbit gently around the current hiding obstacle.
    if (currently_hidden) {
        auto best_obs = CombatGeometry::bestHidingObstacle(me->x, me->y, enemy->x, enemy->y, obstacles);
        if (best_obs.has_value()) {
            const Obstacle& o = best_obs.value();
            double to_obs_x = o.cx - me->x;
            double to_obs_y = o.cy - me->y;
            double tan_x = -to_obs_y;
            double tan_y =  to_obs_x;
            double tlen = std::hypot(tan_x, tan_y);
            if (tlen > 1e-9) {
                tan_x /= tlen;
                tan_y /= tlen;
            }
            const double dot = tan_x * (laser_x - me->x) + tan_y * (laser_y - me->y);
            if (dot > 0.0) {
                tan_x = -tan_x;
                tan_y = -tan_y;
            }

            const double orbit_wx = me->x + 25.0 * tan_x;
            const double orbit_wy = me->y + 25.0 * tan_y;
            const double bear = std::atan2(laser_y - orbit_wy, laser_x - orbit_wx);
            const double orb_red_x = orbit_wx + sep_half * std::cos(bear);
            const double orb_red_y = orbit_wy + sep_half * std::sin(bear);

            const bool orbit_safe = !CombatGeometry::lineOfSightClear(laser_x, laser_y,
                                                                      orb_red_x, orb_red_y,
                                                                      obstacles,
                                                                      10.0);
            if (orbit_safe) {
                res.cmd = follower_->follow(me->x, me->y, me->theta,
                                            {orbit_wx, orbit_wy},
                                            5.0, 2.0, 0.3, v_max * 0.5);
                return res;
            }
        }
    }

    // 2) Reuse cached goal if it is still valid and still hides from the enemy.
    if (cached_goal_.has_value()) {
        const double bear = std::atan2(laser_y - cached_goal_->second, laser_x - cached_goal_->first);
        const double cg_red_x = cached_goal_->first  + sep_half * std::cos(bear);
        const double cg_red_y = cached_goal_->second + sep_half * std::sin(bear);
        const bool cached_hidden = !CombatGeometry::lineOfSightClear(laser_x, laser_y,
                                                                     cg_red_x, cg_red_y,
                                                                     obstacles,
                                                                     10.0);
        if (cached_hidden) {
            auto sc = GridUtils::safePixToFreeCell(grid, me->x, me->y, cell_px);
            auto gc = GridUtils::safePixToFreeCell(grid, cached_goal_->first, cached_goal_->second, cell_px);
            if (sc.has_value() && gc.has_value()) {
                path = planner_->plan(grid, sc.value(), gc.value());
                if (path.has_value() && path->size() >= 2) {
                    res.hiding_point = cached_goal_;
                }
            }
        } else {
            cached_goal_ = std::nullopt;
        }
    }

    // 3) Primary hiding logic using the best blocking obstacle.
    if (!path.has_value()) {
        auto best_obs = CombatGeometry::bestHidingObstacle(me->x, me->y, enemy->x, enemy->y, obstacles);
        if (best_obs.has_value()) {
            const auto hp = CombatGeometry::hidingPointBehindObstacle(me->x, me->y,
                                                                      enemy->x, enemy->y,
                                                                      best_obs.value(),
                                                                      robot_half_length_px + 15.0);
            const double bear = std::atan2(laser_y - hp.second, laser_x - hp.first);
            const double hp_red_x = hp.first  + sep_half * std::cos(bear);
            const double hp_red_y = hp.second + sep_half * std::sin(bear);
            const bool hp_hidden = !CombatGeometry::lineOfSightClear(laser_x, laser_y,
                                                                     hp_red_x, hp_red_y,
                                                                     obstacles,
                                                                     10.0);
            if (hp_hidden) {
                auto sc = GridUtils::safePixToFreeCell(grid, me->x, me->y, cell_px);
                auto gc = GridUtils::safePixToFreeCell(grid, hp.first, hp.second, cell_px);
                if (sc.has_value() && gc.has_value()) {
                    path = planner_->plan(grid, sc.value(), gc.value());
                    if (path.has_value() && path->size() >= 2) {
                        res.hiding_point = hp;
                        cached_goal_ = hp;
                    }
                }
            }
        }
    }

    // 4) Systematic search over random free cells if no direct cover path exists.
    if (!path.has_value() || path->size() < 2 || dec.tactic == "FLEE") {
        auto sc = GridUtils::safePixToFreeCell(grid, me->x, me->y, cell_px);
        if (!sc.has_value()) return res;

        std::mt19937 rng(0);
        std::uniform_int_distribution<int> row_dist(0, static_cast<int>(grid.size()) - 1);
        std::uniform_int_distribution<int> col_dist(0, static_cast<int>(grid[0].size()) - 1);

        double best_score = -1e9;
        for (int k = 0; k < samples; ++k) {
            const int row = row_dist(rng);
            const int col = col_dist(rng);
            if (grid[row][col]) continue;

            auto pix = GridUtils::cellToPix(row, col, cell_px);
            const double cand_x = pix.first;
            const double cand_y = pix.second;
            const double score_dist = std::hypot(cand_x - enemy->x, cand_y - enemy->y)
                                    - 0.15 * std::hypot(cand_x - me->x, cand_y - me->y);
            double score = score_dist;
            if (dec.tactic == "HIDE" || dec.tactic == "ORBIT_HIDE") {
                score += 80.0 * CombatGeometry::countBlockingObstacles(cand_x, cand_y,
                                                                       enemy->x, enemy->y,
                                                                       obstacles,
                                                                       10.0);
            }

            auto cand_path = planner_->plan(grid, sc.value(), {row, col});
            if (cand_path.has_value() && score > best_score) {
                best_score = score;
                path = cand_path;
                res.hiding_point = {cand_x, cand_y};
                cached_goal_ = {cand_x, cand_y};
            }
        }

        // Last resort: flee directly away from the enemy laser.
        if (!path.has_value()) {
            const double away_x = me->x + 2.0 * (me->x - laser_x);
            const double away_y = me->y + 2.0 * (me->y - laser_y);
            res.cmd = follower_->follow(me->x, me->y, me->theta,
                                        {away_x, away_y},
                                        5.0, 2.0, 0.6, v_max);
            return res;
        }
    }

    // 5) Follow the chosen path.
    const int idx = std::min(static_cast<int>(path->size()) - 1, dyn_look);
    const auto [row, col] = (*path)[idx];
    auto wp = GridUtils::cellToPix(row, col, cell_px);

    const double dist_to_goal = res.hiding_point.has_value()
        ? std::hypot(me->x - res.hiding_point->first, me->y - res.hiding_point->second)
        : 1e9;
    const bool near_hiding_point = (dist_to_goal < 3.0 * robot_half_length_px);

    const double to_wp_x = wp.first - me->x;
    const double to_wp_y = wp.second - me->y;
    const double heading_err = std::fmod(std::atan2(to_wp_y, to_wp_x) - me->theta + M_PI,
                                         2.0 * M_PI) - M_PI;

    const bool red_exposed = CombatGeometry::lineOfSightClear(laser_x, laser_y,
                                                              red_x_now, red_y_now,
                                                              obstacles,
                                                              10.0);
    const bool use_reverse = near_hiding_point && (std::fabs(heading_err) > M_PI / 2.0 || red_exposed);

    if (use_reverse) {
        Command c = follower_->follow(me->x, me->y, me->theta + M_PI,
                                      wp,
                                      5.0, 2.0, 0.6,
                                      std::min(v_scaled, v_max * 0.75));
        const double tmp = c.left;
        c.left = -c.right;
        c.right = -tmp;
        res.cmd = c;
    } else {
        res.cmd = follower_->follow(me->x, me->y, me->theta,
                                    wp,
                                    5.0, 2.0, 0.6,
                                    v_scaled);
    }

    res.path = path;
    res.cmd.laser = false;
    return res;
}