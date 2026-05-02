#pragma once
#include "Types.h"
#include <fstream>
#include <string>

struct StrategyDebugInfo {
    int num_tracks = 0;
    int num_obstacles = 0;
    int my_id = -1;
    int enemy_id = -1;
    bool offense_mode = true;
    bool boundary_guard = false;
    double my_x = -1.0;
    double my_y = -1.0;
    double my_theta = 0.0;
    double enemy_x = -1.0;
    double enemy_y = -1.0;
    double enemy_dist_px = -1.0;
    double enemy_bearing_deg = 0.0;
    double nearest_obstacle_dist_px = -1.0;
};

class RunLogger {
public:
    RunLogger();
    bool open(const std::string& filename);
    void log(double t, int frame, const Command& cmd,
             double loop_ms, double fps, const StrategyDebugInfo& dbg);
    void close();
    bool isOpen() const;
private:
    std::ofstream file_;
};
