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

// Scripted attacker - moves toward center/target
class ScriptedAttacker : public OpponentBehavior {
private:
    double target_x_, target_y_;
    
public:
    ScriptedAttacker(double tx = 320, double ty = 240) 
        : target_x_(tx), target_y_(ty) {}
    
    Command getCommand(double now, double x, double y, double theta) override {
        // Calculate angle to target
        double dx = target_x_ - x;
        double dy = target_y_ - y;
        double target_angle = std::atan2(dy, dx);
        
        // Angle difference
        double angle_diff = target_angle - theta;
        
        // Normalize to [-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2*M_PI;
        while (angle_diff < -M_PI) angle_diff += 2*M_PI;
        
        // Simple proportional control
        double turn_rate = angle_diff * 0.5;  // P-controller
        double forward_speed = 0.5;
        
        // Differential drive
        double left = forward_speed - turn_rate;
        double right = forward_speed + turn_rate;
        
        // Clamp
        left = std::max(-1.0, std::min(1.0, left));
        right = std::max(-1.0, std::min(1.0, right));
        
        return Command{ left, right, false };
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
