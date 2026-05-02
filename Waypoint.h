#pragma once
#include "Types.h"
#include <utility>
#include <cmath>

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

// Transform marker centroid to front-axle position (kinematic control point).
// The front axle is AHEAD of the marker centroid by marker_offset_x pixels.
inline void getWheelCenterPosition(double marker_x, double marker_y, double theta,
                                   double& wheel_x, double& wheel_y,
                                   double marker_offset_x = 31.0) {
    wheel_x = marker_x + marker_offset_x * std::cos(theta);
    wheel_y = marker_y + marker_offset_x * std::sin(theta);
}
