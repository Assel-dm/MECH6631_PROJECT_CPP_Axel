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
