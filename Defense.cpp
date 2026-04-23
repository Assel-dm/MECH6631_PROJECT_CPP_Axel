#include "Defense.h"
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

static const Obstacle* best_hiding_obstacle(
    const RobotTrack& me, const RobotTrack& enemy, const std::vector<Obstacle>& obs)
{
    double laser_x = enemy.x + (enemy.sep_px * 0.5) * std::cos(enemy.theta);
    double laser_y = enemy.y + (enemy.sep_px * 0.5) * std::sin(enemy.theta);

    const Obstacle* best = nullptr;
    double best_score = -1e9;

    auto line_dist = [&](double px, double py) {
        double ax = me.x, ay = me.y, bx = laser_x, by = laser_y;
        double abx = bx-ax, aby = by-ay, apx = px-ax, apy = py-ay;
        double ab2 = abx*abx + aby*aby;
        if (ab2 < 1e-9) return std::hypot(px-ax, py-ay);
        double t = clamp((apx*abx + apy*aby) / ab2, 0.0, 1.0);
        return std::hypot(px - (ax+t*abx), py - (ay+t*aby));
    };

    for (auto& o : obs) {
        if (std::max(o.w, o.h) > 200) continue;
        double r      = 0.5 * std::hypot(o.w, o.h) + 12.0;
        double dline  = line_dist(o.cx, o.cy);
        double d_me   = std::hypot(me.x - o.cx, me.y - o.cy);
        double d_en   = std::hypot(enemy.x - o.cx, enemy.y - o.cy);
        double score  = (dline <= r ? 200.0 : 0.0) + 0.15*d_en - 0.15*d_me;
        if (score > best_score) { best_score = score; best = &o; }
    }
    return best;
}

static std::pair<double,double> hiding_point_behind_obstacle(
    double laser_x, double laser_y, const Obstacle& o,
    double stand_off_px, double extra_offset_px)
{
    double vx = o.cx - laser_x, vy = o.cy - laser_y;
    double n = std::hypot(vx, vy);
    if (n < 1e-9) return { o.cx, o.cy };
    double ux = vx/n, uy = vy/n;
    double radius = 0.5 * std::hypot(o.w, o.h);
    return { o.cx + (radius + stand_off_px + extra_offset_px) * ux,
             o.cy + (radius + stand_off_px + extra_offset_px) * uy };
}

static bool has_los(const Grid& gv, int cell_px,
    double x1, double y1, double x2, double y2)
{
    int r1=(int)(y1/cell_px), c1=(int)(x1/cell_px);
    int r2=(int)(y2/cell_px), c2=(int)(x2/cell_px);
    int dr=std::abs(r2-r1), dc=std::abs(c2-c1);
    int sr=(r1<r2)?1:-1, sc=(c1<c2)?1:-1;
    int err=dr-dc, r=r1, c=c1;
    int rows=(int)gv.size(), cols=(int)gv[0].size();
    while (true) {
        if (r<0||r>=rows||c<0||c>=cols) return false;
        if (gv[r][c]) return false;
        if (r==r2&&c==c2) return true;
        int e2=2*err;
        if (e2>-dc){err-=dc; r+=sr;}
        if (e2< dr){err+=dr; c+=sc;}
    }
}

DefenseStrategy::DefenseStrategy(AStarPlanner* p, WaypointFollower* f, FuzzyLogic* fl)
    : planner_(p), follower_(f), fuzzy_(fl) {}

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
    Result res;
    res.cmd = { 0.0, 0.0, false };
    res.path = std::nullopt;
    res.hiding_point = std::nullopt;

    const RobotTrack* me    = find_me(tracks, my_id);
    const RobotTrack* enemy = pick_nearest_enemy(tracks, my_id);
    if (!me || !enemy) return res;

    // Enemy laser originates from their BLUE (front) marker
    double laser_x = enemy->x + (enemy->sep_px * 0.5) * std::cos(enemy->theta);
    double laser_y = enemy->y + (enemy->sep_px * 0.5) * std::sin(enemy->theta);

    TacticalFeatures feat = fuzzy_->extractFeatures(*me, *enemy, obstacles);
    FuzzyDecision    dec  = fuzzy_->defense(feat);
    res.decision = dec;

    double v_scaled = v_max * dec.speed_scale;
    int dyn_look    = std::max(1, (int)std::round(lookahead_cells * dec.lookahead_scale));

    double sep_half  = me->sep_px * 0.5;
    double red_x_now = me->x - sep_half * std::cos(me->theta);
    double red_y_now = me->y - sep_half * std::sin(me->theta);

    auto snap_to_free = [&](std::pair<int,int> cell) -> std::pair<int,int> {
        int rows = (int)grid.size(), cols = (int)grid[0].size();
        cell.first  = std::max(0, std::min(rows-1, cell.first));
        cell.second = std::max(0, std::min(cols-1, cell.second));
        if (!grid[cell.first][cell.second]) return cell;
        for (int rad=1; rad<8; rad++)
            for (int dy=-rad; dy<=rad; dy++)
                for (int dx=-rad; dx<=rad; dx++) {
                    int ny=cell.first+dy, nx=cell.second+dx;
                    if (ny>=0&&ny<rows&&nx>=0&&nx<cols&&!grid[ny][nx])
                        return {ny,nx};
                }
        return cell;
    };

    // Already hidden from laser — orbit slowly to maintain laser cover
    bool currently_hidden = !has_los(grid_visual, cell_px, laser_x, laser_y, red_x_now, red_y_now);
    if (currently_hidden) {
        const Obstacle* o = best_hiding_obstacle(*me, *enemy, obstacles);
        if (o) {
            double to_obs_x = o->cx - me->x, to_obs_y = o->cy - me->y;
            double tan_x = -to_obs_y,        tan_y    =  to_obs_x;
            double tlen  = std::hypot(tan_x, tan_y);
            if (tlen > 1e-9) { tan_x /= tlen; tan_y /= tlen; }
            double dot = tan_x*(laser_x-me->x) + tan_y*(laser_y-me->y);
            if (dot > 0) { tan_x=-tan_x; tan_y=-tan_y; }

            double orbit_wx = me->x + tan_x * 25.0;
            double orbit_wy = me->y + tan_y * 25.0;

            double bear       = std::atan2(laser_y - orbit_wy, laser_x - orbit_wx);
            double orb_red_x  = orbit_wx + sep_half * std::cos(bear);
            double orb_red_y  = orbit_wy + sep_half * std::sin(bear);
            bool orbit_safe   = !has_los(grid_visual, cell_px, laser_x, laser_y, orb_red_x, orb_red_y);

            if (orbit_safe) {
                res.cmd = follower_->follow(me->x, me->y, me->theta,
                    { orbit_wx, orbit_wy }, 5.0, 2.0, 0.3, v_max * 0.5);  // ← raised from 0.2 to 0.5
            }
        }
        res.cmd.laser = false;
        return res;
    }

    // If cached goal still hides red marker, reuse it without replanning
    std::optional<std::vector<std::pair<int,int>>> path;

    if (cached_goal_.has_value()) {
        double bear  = std::atan2(laser_y - cached_goal_->second, laser_x - cached_goal_->first);
        double cg_rx = cached_goal_->first  + sep_half * std::cos(bear);
        double cg_ry = cached_goal_->second + sep_half * std::sin(bear);
        if (!has_los(grid_visual, cell_px, laser_x, laser_y, cg_rx, cg_ry)) {
            res.hiding_point = cached_goal_;
            auto sc = snap_to_free({ (int)(me->y/cell_px), (int)(me->x/cell_px) });
            auto gc = snap_to_free({ (int)(cached_goal_->second/cell_px),
                                     (int)(cached_goal_->first /cell_px) });
            path = planner_->plan(grid, sc, gc);
        } else {
            cached_goal_ = std::nullopt;  // cached goal no longer hides from laser
        }
    }

    // Primary hiding logic — always attempt, not gated on fuzzy decision
    if (!path) {
        const Obstacle* o = best_hiding_obstacle(*me, *enemy, obstacles);
        if (o) {
            double stand_off = robot_half_length_px + 15.0;
            double extra     = sep_half;
            auto hp = hiding_point_behind_obstacle(laser_x, laser_y, *o, stand_off, extra);
            double bear  = std::atan2(laser_y - hp.second, laser_x - hp.first);
            double hp_rx = hp.first  + extra * std::cos(bear);
            double hp_ry = hp.second + extra * std::sin(bear);
            if (!has_los(grid_visual, cell_px, laser_x, laser_y, hp_rx, hp_ry)) {
                res.hiding_point = hp;
                cached_goal_     = hp;
                auto sc = snap_to_free({ (int)(me->y/cell_px), (int)(me->x/cell_px) });
                auto gc = snap_to_free({ (int)(hp.second/cell_px), (int)(hp.first/cell_px) });
                path = planner_->plan(grid, sc, gc);
            }
        }
    }

    // Systematic search — laser-hidden candidates only (no fallback to exposed positions)
    if (!path || path->size() < 2) {
        auto sc = snap_to_free({ (int)(me->y/cell_px), (int)(me->x/cell_px) });
        double best_score = -1e9;

        for (const auto& o : obstacles) {
            if (std::max(o.w, o.h) > 200) continue;
            double obs_r = 0.5 * std::hypot(o.w, o.h);

            for (int a = 0; a < 8; a++) {
                double angle  = a * (2.0 * M_PI / 8.0);
                double cand_x = o.cx + (obs_r + robot_half_length_px + sep_half + 10.0) * std::cos(angle);
                double cand_y = o.cy + (obs_r + robot_half_length_px + sep_half + 10.0) * std::sin(angle);

                int gy = (int)(cand_y/cell_px), gx = (int)(cand_x/cell_px);
                if (gy<0||gy>=(int)grid.size()||gx<0||gx>=(int)grid[0].size()) continue;
                if (grid_visual[gy][gx] || grid[gy][gx]) continue;

                double bear   = std::atan2(laser_y-cand_y, laser_x-cand_x);
                double red_cx = cand_x + sep_half * std::cos(bear);
                double red_cy = cand_y + sep_half * std::sin(bear);
                bool hidden   = !has_los(grid_visual, cell_px, laser_x, laser_y, red_cx, red_cy);

                // Only consider positions that actually hide the red marker from the laser
                if (!hidden) continue;

                double score = 300.0 - 0.15 * std::hypot(cand_x-me->x, cand_y-me->y);

                auto gc        = snap_to_free({ gy, gx });
                auto cand_path = planner_->plan(grid, sc, gc);
                if (cand_path && score > best_score) {
                    best_score       = score;
                    path             = cand_path;
                    res.hiding_point = { cand_x, cand_y };
                    cached_goal_     = { cand_x, cand_y };
                }
            }
        }

        // Last resort: no laser-safe cover found — flee directly away from laser
        if (!path) {
            double away_x = me->x + (me->x - laser_x) * 2.0;
            double away_y = me->y + (me->y - laser_y) * 2.0;
            res.cmd = follower_->follow(me->x, me->y, me->theta,
                { away_x, away_y }, 5.0, 2.0, 0.6, v_max);
            res.cmd.laser = false;
            return res;
        }
    }

    // Follow path toward laser-safe cover
    int idx = std::min((int)path->size()-1, dyn_look);
    auto [wy, wx] = (*path)[idx];
    double wx_pix = (wx + 0.5) * cell_px;
    double wy_pix = (wy + 0.5) * cell_px;

    double red_x = me->x - sep_half * std::cos(me->theta);
    double red_y = me->y - sep_half * std::sin(me->theta);
    bool red_exposed = has_los(grid_visual, cell_px, laser_x, laser_y, red_x, red_y);

    double dist_to_goal = res.hiding_point.has_value()
        ? std::hypot(me->x - res.hiding_point->first, me->y - res.hiding_point->second)
        : 1e9;
    bool near_hiding_point = (dist_to_goal < robot_half_length_px * 3.0);

    double to_wp_x = wx_pix - me->x, to_wp_y = wy_pix - me->y;
    double heading_err = std::fmod(std::atan2(to_wp_y, to_wp_x) - me->theta + M_PI, 2.0*M_PI) - M_PI;

    // Allow reverse when near cover if heading is wrong or red marker is still exposed
    bool use_reverse = near_hiding_point
                    && (std::fabs(heading_err) > M_PI/2.0 || red_exposed);

    double effective_v = use_reverse ? std::min(v_scaled, v_max * 0.75) : v_scaled;  // raised from 0.55

    if (use_reverse) {
        res.cmd = follower_->follow(me->x, me->y, me->theta + M_PI,
            { wx_pix, wy_pix }, 5.0, 2.0, 0.6, effective_v);
        double tmp = res.cmd.left;
        res.cmd.left  = -res.cmd.right;
        res.cmd.right = -tmp;
    } else {
        res.cmd = follower_->follow(me->x, me->y, me->theta,
            { wx_pix, wy_pix }, 5.0, 2.0, 0.6, v_scaled);
    }

    res.cmd.laser = false;
    res.path = path;
    return res;
}