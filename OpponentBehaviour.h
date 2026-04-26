#pragma once

#include "Types.h"
#include <random>

// Base class for opponent test behaviors
class OpponentBehavior {
public:
    virtual ~OpponentBehavior() = default;
    
    // Returns command for opponent robot
    virtual Command getCommand(double now, double x, double y, double theta) = 0;
};

// Random movement - good for obstacle avoidance testing
class RandomMovement : public OpponentBehavior {
private:
    std::mt19937 rng_;
    double change_time_;
    double last_change_;
    double vel_left_, vel_right_;
    
public:
    RandomMovement() 
        : rng_(std::random_device{}())
        , change_time_(2.0)  // Change direction every 2 seconds
        , last_change_(0.0)
        , vel_left_(0.0)
        , vel_right_(0.0)
    {}
    
    Command getCommand(double now, double x, double y, double theta) override {
        // Change direction periodically
        if (now - last_change_ > change_time_) {
            std::uniform_real_distribution<> dist(-0.7, 0.7);
            vel_left_ = dist(rng_);
            vel_right_ = dist(rng_);
            last_change_ = now;
        }
        
        return Command{ vel_left_, vel_right_, false };
    }
};

// Hunting attacker — chases the defender and fires laser on clear LOS
class ScriptedAttacker : public OpponentBehavior {
private:
    double alpha_max_;
    int    los_stable_frames_ = 0;
    bool   laser_fired_       = false;
    const int LOS_FRAMES_REQ  = 6;

public:
    ScriptedAttacker(double /*unused_tx*/ = 0, double /*unused_ty*/ = 0,
                     double alpha_max = M_PI / 2.0)
        : alpha_max_(alpha_max) {}

    // Call this overload each frame from the main loop
    Command getCommandFull(
        double now, double x, double y, double theta,
        double target_x, double target_y,
        const Grid& grid, int cell_px, double ax_offset,
        int& pw_laser_out)
    {
        // --- Drive toward target (your robot) ---
        double dx = target_x - x;
        double dy = target_y - y;
        double target_angle = std::atan2(dy, dx);

        double angle_diff = target_angle - theta;
        while (angle_diff >  M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        double turn_rate     = angle_diff * 0.6;
        double forward_speed = 0.7 * (1.0 - std::min(1.0, std::abs(angle_diff) / M_PI));
        double left  = std::max(-1.0, std::min(1.0, forward_speed - turn_rate));
        double right = std::max(-1.0, std::min(1.0, forward_speed + turn_rate));

        // --- Laser mount position ---
        double laser_x = x + ax_offset * std::cos(theta);
        double laser_y = y + ax_offset * std::sin(theta);

        // --- Laser servo ---
        double bearing    = std::atan2(target_y - laser_y, target_x - laser_x);
        double rel_angle  = std::fmod(bearing - theta + M_PI, 2.0 * M_PI) - M_PI;
        rel_angle         = std::max(-alpha_max_, std::min(alpha_max_, rel_angle));
        pw_laser_out      = 1500 + (int)(rel_angle / alpha_max_ * 500.0);

        // --- LOS raycast ---
        bool los_clear = hasLOS(laser_x, laser_y, target_x, target_y, grid, cell_px);
        double dist    = std::hypot(laser_x - target_x, laser_y - target_y);

        if (los_clear && !laser_fired_ && dist < 900.0)
            los_stable_frames_++;
        else
            los_stable_frames_ = 0;

        bool fire = (!laser_fired_ && los_stable_frames_ >= LOS_FRAMES_REQ);
        if (fire) laser_fired_ = true;

        return Command{ left, right, fire };
    }

    // Fallback for base interface (not used in TEST_DEFENSE)
    Command getCommand(double, double, double, double) override {
        return Command{ 0.5, 0.5, false };
    }

private:
    bool hasLOS(double x1, double y1, double x2, double y2,
                const Grid& grid, int cell_px) const
    {
        int r1 = (int)(y1/cell_px), c1 = (int)(x1/cell_px);
        int r2 = (int)(y2/cell_px), c2 = (int)(x2/cell_px);
        int dr = std::abs(r2-r1), dc = std::abs(c2-c1);
        int sr = (r1<r2)?1:-1,    sc = (c1<c2)?1:-1;
        int err = dr-dc, r = r1, c = c1;
        int rows = (int)grid.size(), cols = (int)grid[0].size();
        while (true) {
            if (r<0||r>=rows||c<0||c>=cols) return false;
            if (grid[r][c]) return false;
            if (r==r2&&c==c2) return true;
            int e2 = 2*err;
            if (e2>-dc){err-=dc; r+=sr;}
            if (e2< dr){err+=dr; c+=sc;}
        }
    }
};

// Scripted defender - stays in defensive zone, blocks target
class ScriptedDefender : public OpponentBehavior {
private:
    double defend_x_, defend_y_;
    double defend_radius_;
    
public:
    ScriptedDefender(double dx = 480, double dy = 240, double radius = 100) 
        : defend_x_(dx), defend_y_(dy), defend_radius_(radius) {}
    
    Command getCommand(double now, double x, double y, double theta) override {
        // Stay within defensive radius
        double dx = x - defend_x_;
        double dy = y - defend_y_;
        double dist = std::hypot(dx, dy);
        
        if (dist > defend_radius_) {
            // Return to defensive position
            double target_angle = std::atan2(-dy, -dx);
            double angle_diff = target_angle - theta;
            
            while (angle_diff > M_PI) angle_diff -= 2*M_PI;
            while (angle_diff < -M_PI) angle_diff += 2*M_PI;
            
            double turn_rate = angle_diff * 0.5;
            double forward_speed = 0.4;
            
            double left = forward_speed - turn_rate;
            double right = forward_speed + turn_rate;
            
            left = std::max(-1.0, std::min(1.0, left));
            right = std::max(-1.0, std::min(1.0, right));
            
            return Command{ left, right, false };
        } else {
            // Patrol in circle
            return Command{ 0.3, -0.3, false };  // Spin slowly
        }
    }
};

// Patroller - follows waypoints
class Patroller : public OpponentBehavior {
private:
    std::vector<std::pair<double, double>> waypoints_;
    size_t current_waypoint_;
    double waypoint_threshold_;
    
public:
    Patroller(std::vector<std::pair<double, double>> wps)
        : waypoints_(wps), current_waypoint_(0), waypoint_threshold_(50.0) {}
    
    Command getCommand(double now, double x, double y, double theta) override {
        if (waypoints_.empty()) return Command{ 0.0, 0.0, false };
        
        // Get current target waypoint
        auto [tx, ty] = waypoints_[current_waypoint_];
        
        // Check if reached
        double dist = std::hypot(tx - x, ty - y);
        if (dist < waypoint_threshold_) {
            current_waypoint_ = (current_waypoint_ + 1) % waypoints_.size();
            return Command{ 0.0, 0.0, false };  // Stop briefly
        }
        
        // Move toward waypoint
        double target_angle = std::atan2(ty - y, tx - x);
        double angle_diff = target_angle - theta;
        
        while (angle_diff > M_PI) angle_diff -= 2*M_PI;
        while (angle_diff < -M_PI) angle_diff += 2*M_PI;
        
        double turn_rate = angle_diff * 0.5;
        double forward_speed = 0.5;
        
        double left = forward_speed - turn_rate;
        double right = forward_speed + turn_rate;
        
        left = std::max(-1.0, std::min(1.0, left));
        right = std::max(-1.0, std::min(1.0, right));
        
        return Command{ left, right, false };
    }
};
