#include "Fuzzy.h"
#include <cmath>
#include <algorithm>

// ---------------------- Helper membership functions ----------------------

static double grade(double x, double x0, double x1) {
    if (x <= x0) return 0.0;
    if (x >= x1) return 1.0;
    return (x - x0) / std::max(1e-9, x1 - x0);
}

static double reverse_grade(double x, double x0, double x1) {
    return 1.0 - grade(x, x0, x1);
}

static double triangle(double x, double a, double b, double c) {
    if (x <= a || x >= c) return 0.0;
    if (x == b) return 1.0;
    if (x < b) return (x - a) / std::max(1e-9, b - a);
    return (c - x) / std::max(1e-9, c - b);
}

// ---------------------- Helper tactical functions ----------------------

static double nearest_obstacle_distance(const RobotTrack& me,
    const std::vector<Obstacle>& obs)
{
    if (obs.empty()) return 1e9;
    double best = 1e9;
    for (auto& o : obs) {
        double d = std::hypot(me.x - o.cx, me.y - o.cy);
        if (d < best) best = d;
    }
    return best;
}

static int count_blocking_obstacles(const RobotTrack& me,
    const RobotTrack& enemy,
    const std::vector<Obstacle>& obs)
{
    auto line_dist = [&](double px, double py) {
        double ax = me.x, ay = me.y;
        double bx = enemy.x, by = enemy.y;

        double abx = bx - ax;
        double aby = by - ay;
        double apx = px - ax;
        double apy = py - ay;

        double ab2 = abx * abx + aby * aby;
        if (ab2 < 1e-9) return std::hypot(px - ax, py - ay);

        double t = (apx * abx + apy * aby) / ab2;
        t = clamp(t, 0.0, 1.0);

        double qx = ax + t * abx;
        double qy = ay + t * aby;

        return std::hypot(px - qx, py - qy);
        };

    int c = 0;
    for (auto& o : obs) {
        double r = 0.5 * std::hypot(o.w, o.h) + 10.0;
        double d = line_dist(o.cx, o.cy);
        if (d <= r) c++;
    }
    return c;
}

// ---------------------- Feature extraction ----------------------

TacticalFeatures FuzzyLogic::extractFeatures(
    const RobotTrack& me,
    const RobotTrack& enemy,
    const std::vector<Obstacle>& obs)
{
    double dx = enemy.x - me.x;
    double dy = enemy.y - me.y;

    double enemy_dist = std::hypot(dx, dy);
    double desired = std::atan2(dy, dx);
    double err = angle_wrap(desired - me.theta);
    double enemy_bearing_deg = err * 180.0 / M_PI;

    double nearest_obs_dist = nearest_obstacle_distance(me, obs);
    int n_block = count_blocking_obstacles(me, enemy, obs);

    double has_cover = grade((double)n_block, 0.5, 1.5);
    double close_danger = reverse_grade(enemy_dist, 120.0, 260.0);
    double obstacle_pressure = reverse_grade(nearest_obs_dist, 80.0, 220.0);

    double clutter = grade((double)obs.size(), 1.0, 4.0);
    double medium_range = triangle(enemy_dist, 120.0, 220.0, 380.0);

    double surprise_desire = std::min(1.0, 0.6 * clutter + 0.7 * medium_range);

    return {
        enemy_dist,
        enemy_bearing_deg,
        nearest_obs_dist,
        n_block,
        has_cover,
        close_danger,
        obstacle_pressure,
        surprise_desire
    };
}

// ---------------------- Offense fuzzy logic ----------------------

FuzzyDecision FuzzyLogic::offense(const TacticalFeatures& f)
{
    double near_enemy = reverse_grade(f.enemy_dist, 120.0, 220.0);
    double med_enemy = triangle(f.enemy_dist, 120.0, 240.0, 420.0);
    double far_enemy = grade(f.enemy_dist, 260.0, 420.0);

    double attack_direct = std::max(near_enemy, 0.6 * far_enemy);
    double attack_alt = std::min(med_enemy, f.surprise_desire);

    double cautious = f.obstacle_pressure;
    double aggressive = std::max(near_enemy, far_enemy * 0.7);

    FuzzyDecision d;

    d.tactic = (attack_alt > attack_direct)
        ? "ATTACK_ALT"
        : "ATTACK_DIRECT";

    d.speed_scale = clamp(
        0.75 + 0.45 * aggressive - 0.30 * cautious,
        0.45, 1.25
    );

    d.lookahead_scale = clamp(
        1.10 - 0.35 * cautious + 0.20 * far_enemy,
        0.75, 1.45
    );

    d.prefer_alt_path = (d.tactic == "ATTACK_ALT");
    d.use_hiding_obstacle = false;

    d.debug["attack_direct"] = attack_direct;
    d.debug["attack_alt"] = attack_alt;

    return d;
}

// ---------------------- Defense fuzzy logic ----------------------

FuzzyDecision FuzzyLogic::defense(const TacticalFeatures& f)
{
    double enemy_too_close = reverse_grade(f.enemy_dist, 90.0, 180.0);
    double enemy_close = reverse_grade(f.enemy_dist, 150.0, 280.0);

    double can_hide = f.has_cover;
    double pressure = std::max(f.close_danger, 0.7 * f.obstacle_pressure);

    double flee_strength = enemy_too_close;
    double hide_strength = std::min(enemy_close, can_hide);
    double orbit_strength = std::min(can_hide, 1.0 - enemy_too_close);

    FuzzyDecision d;

    if (flee_strength > std::max(hide_strength, orbit_strength)) {
        d.tactic = "FLEE";
    }
    else if (orbit_strength > hide_strength) {
        d.tactic = "ORBIT_HIDE";
    }
    else {
        d.tactic = "HIDE";
    }

    d.speed_scale = clamp(
        0.55 + 0.80 * enemy_close,  // raised floor: was 0.45 + 0.90
        0.55, 1.35                  // minimum raised from 0.35 → 0.55
    );

    d.lookahead_scale = clamp(
        1.20 - 0.55 * pressure,
        0.70, 1.25
    );

    d.prefer_alt_path = (d.tactic == "HIDE" || d.tactic == "ORBIT_HIDE");
    d.use_hiding_obstacle = d.prefer_alt_path;

    d.debug["flee_strength"] = flee_strength;
    d.debug["hide_strength"] = hide_strength;
    d.debug["orbit_strength"] = orbit_strength;

    return d;
}