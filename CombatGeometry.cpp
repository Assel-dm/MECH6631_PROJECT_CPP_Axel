#include "CombatGeometry.h"

#include <cmath>
#include <limits>

namespace CombatGeometry {

double pointLineDistance(double px, double py,
                         double ax, double ay,
                         double bx, double by)
{
    const double abx = bx - ax;
    const double aby = by - ay;
    const double apx = px - ax;
    const double apy = py - ay;

    const double ab2 = abx * abx + aby * aby;
    if (ab2 < 1e-9) return std::hypot(px - ax, py - ay);

    double t = (apx * abx + apy * aby) / ab2;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    const double qx = ax + t * abx;
    const double qy = ay + t * aby;
    return std::hypot(px - qx, py - qy);
}

bool obstacleBlocksLine(double my_x, double my_y,
                        double enemy_x, double enemy_y,
                        const Obstacle& obs,
                        double margin_px)
{
    const double radius = 0.5 * std::hypot(static_cast<double>(obs.w),
                                           static_cast<double>(obs.h)) + margin_px;
    const double d = pointLineDistance(obs.cx, obs.cy, my_x, my_y, enemy_x, enemy_y);
    return d <= radius;
}

bool lineOfSightClear(double my_x, double my_y,
                      double enemy_x, double enemy_y,
                      const std::vector<Obstacle>& obstacles,
                      double margin_px)
{
    for (const auto& o : obstacles) {
        if (obstacleBlocksLine(my_x, my_y, enemy_x, enemy_y, o, margin_px)) {
            return false;
        }
    }
    return true;
}

int countBlockingObstacles(double my_x, double my_y,
                           double enemy_x, double enemy_y,
                           const std::vector<Obstacle>& obstacles,
                           double margin_px)
{
    int c = 0;
    for (const auto& o : obstacles) {
        if (obstacleBlocksLine(my_x, my_y, enemy_x, enemy_y, o, margin_px)) {
            ++c;
        }
    }
    return c;
}

double nearestObstacleDistance(double my_x, double my_y,
                               const std::vector<Obstacle>& obstacles)
{
    if (obstacles.empty()) return 1e9;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& o : obstacles) {
        const double d = std::hypot(my_x - o.cx, my_y - o.cy);
        if (d < best) best = d;
    }
    return std::isfinite(best) ? best : 1e9;
}

std::optional<Obstacle> bestHidingObstacle(double my_x, double my_y,
                                           double enemy_x, double enemy_y,
                                           const std::vector<Obstacle>& obstacles)
{
    if (obstacles.empty()) return std::nullopt;

    double best_score = -1e18;
    std::optional<Obstacle> best;

    for (const auto& o : obstacles) {
        const bool blocks = obstacleBlocksLine(my_x, my_y, enemy_x, enemy_y, o, 12.0);
        const double d_me = std::hypot(my_x - o.cx, my_y - o.cy);
        const double d_en = std::hypot(enemy_x - o.cx, enemy_y - o.cy);

        double score = 0.0;
        if (blocks) score += 200.0;
        score += 0.15 * d_en;
        score -= 0.35 * d_me;

        if (score > best_score) {
            best_score = score;
            best = o;
        }
    }
    return best;
}

std::pair<double,double> hidingPointBehindObstacle(double my_x, double my_y,
                                                   double enemy_x, double enemy_y,
                                                   const Obstacle& obs,
                                                   double stand_off_px)
{
    (void)my_x;
    (void)my_y;

    const double vx = obs.cx - enemy_x;
    const double vy = obs.cy - enemy_y;
    const double n = std::hypot(vx, vy);
    if (n < 1e-9) return {obs.cx, obs.cy};

    const double ux = vx / n;
    const double uy = vy / n;
    const double radius = 0.5 * std::hypot(static_cast<double>(obs.w),
                                           static_cast<double>(obs.h));
    const double tx = obs.cx + (radius + stand_off_px) * ux;
    const double ty = obs.cy + (radius + stand_off_px) * uy;
    return {tx, ty};
}

} // namespace CombatGeometry
