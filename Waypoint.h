#pragma once
#include "Types.h"
#include <utility>

class WaypointFollower {
public:
    Command follow(
        double x, double y, double theta,
        std::pair<double, double> waypoint,
        double stop_dist,
        double k_ang,
        double k_lin,
        double v_max);
};
