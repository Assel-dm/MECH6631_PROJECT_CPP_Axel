// Types.h
#pragma once
#define _USE_MATH_DEFINES
#include <vector>
#include <optional>
#include <cstdint>
#include <map>
#include <string>
#include <cmath>
#include <algorithm>

struct Blob {
    double x, y;
    double area;
};

struct RobotDet {
    Blob front;
    Blob rear;
    double x, y;
    double theta;
    double sep_px;
};

struct RobotTrack {
    int id;
    double x, y;
    double theta;
    double sep_px;
    double last_seen;
    int misses;
};

struct Obstacle {
    int x, y, w, h;
    double cx, cy;
    double area;
};

struct Command {
    double left;
    double right;
    bool laser;
};

using Grid = std::vector<std::vector<uint8_t>>;

// --- small helpers ---
inline double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

inline double angle_wrap(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

inline double dist2d(double x1, double y1, double x2, double y2) {
    return std::hypot(x1 - x2, y1 - y2);
}

// --- Robot control helpers ---
// ⭐ CALIBRATED NEUTRAL VALUES - Tested and confirmed
constexpr int NEUTRAL_LEFT_PW  = 1500;  // Left servo perfect at 1500µs
constexpr int NEUTRAL_RIGHT_PW = 1480;  // Right servo perfect at 1480µs

// Converts velocity to pulse width for LEFT servo
inline int vel_to_pw_left(double vel) {
    constexpr int pw_range  = 500;
    constexpr double deadband = 0.05;  // 5% deadband around zero
    
    if (vel < -1.0) vel = -1.0;
    if (vel >  1.0) vel =  1.0;
    
    // Apply deadband - force small velocities to exactly zero
    if (vel > -deadband && vel < deadband) {
        vel = 0.0;
    }
    
    return NEUTRAL_LEFT_PW + static_cast<int>(vel * pw_range);
}

// Converts velocity to pulse width for RIGHT servo
// ⭐ INVERTED because servos are mirrored (mounted opposite directions)
inline int vel_to_pw_right(double vel) {
    constexpr int pw_range  = 500;
    constexpr double deadband = 0.05;  // 5% deadband around zero
    
    if (vel < -1.0) vel = -1.0;
    if (vel >  1.0) vel =  1.0;
    
    // Apply deadband - force small velocities to exactly zero
    if (vel > -deadband && vel < deadband) {
        vel = 0.0;
    }
    
    // ⭐ INVERT: Negate velocity because right servo is mounted backwards
    return NEUTRAL_RIGHT_PW - static_cast<int>(vel * pw_range);  // Note the MINUS
}

// Legacy function - uses left servo calibration (for backward compatibility)
inline int vel_to_pw(double vel) {
    return vel_to_pw_left(vel);
}

// Strategy mode selection
enum class StrategyMode {
    OFFENSE,  // Attack opponent
    DEFENSE,  // Defend territory
    AUTO      // Decide based on game state (advanced)
};
