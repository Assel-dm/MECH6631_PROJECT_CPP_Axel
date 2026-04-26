#include "Waypoint.h"
#include <cmath>
#include <iostream>

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

    // ⭐⭐⭐ MUCH STRONGER angular velocity
    double w = k_ang * err;  // No scaling down!
    
    // Cap at reasonable max
    double w_max = k_ang * M_PI / 3;  // ±60° equivalent
    if (std::fabs(w) > w_max) {
        w = std::copysign(w_max, w);
    }
    
    double v = 0.0;

    if (distv > stop_dist) {
        double base_v = clamp(k_lin * (distv - stop_dist), 0.0, v_max);
        
        // ⭐⭐⭐ SLOW DOWN when turning sharply
        if (std::fabs(err) > M_PI / 4) {  // > 45°
            v = 0.4 * v_max;  // Slow to 40% when turning hard
        } else if (std::fabs(err) > M_PI / 6) {  // > 30°
            v = 0.6 * v_max;  // Moderate turn
        } else {
            v = std::max(base_v, 0.7 * v_max);  // Full speed when aligned
        }
        
        // Debug output
        static int debug_counter = 0;
        if (debug_counter++ % 50 == 0) {
            std::cout << "[WP] distv=" << distv << " err=" << (err*180/M_PI) 
                      << "° v=" << v << " w=" << w << std::endl;
        }
    }

    // ⭐⭐⭐ MUCH LARGER turning coefficient
    double left = v - 0.5 * w;   // Increased from 0.15 to 0.5!
    double right = v + 0.5 * w;

    // Normalize if needed
    double m = std::max(1.0, std::max(std::fabs(left), std::fabs(right)));
    if (m > 1.0) {
        left /= m;
        right /= m;
    }

    return { clamp(-left, -1.0, 1.0),   // Keep negation for servo
             clamp(right, -1.0, 1.0),
             false };
}