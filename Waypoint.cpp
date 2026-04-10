#include "Waypoint.h"
#include <cmath>

Command WaypointFollower::follow(
    double x, double y, double theta,
    std::pair<double, double> waypoint,
    double stop_dist,
    double k_ang,
    double k_lin,
    double v_max)
{
    double wx = waypoint.first;
    double wy = waypoint.second;

    double dx = wx - x;
    double dy = wy - y;

    double distv = std::hypot(dx, dy);
    double desired = std::atan2(dy, dx);
    double err = angle_wrap(desired - theta);

    double w = k_ang * err;
    double v = 0.0;

    if (distv > stop_dist) {
        v = clamp(k_lin * (distv - stop_dist), 0.0, v_max) *
            std::max(0.0, std::cos(err));
    }

    double left = v - 0.7 * w;
    double right = v + 0.7 * w;

    double m = std::max(1.0, std::max(std::fabs(left), std::fabs(right)));
    left /= m;
    right /= m;

    return { clamp(left, -1.0, 1.0),
            clamp(right, -1.0, 1.0),
            false };
}