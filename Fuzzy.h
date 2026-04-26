#pragma once
#include "Types.h"
#include <map>
#include <string>
#include <vector>

struct TacticalFeatures {
    double enemy_dist;
    double enemy_bearing_deg;
    double nearest_obs_dist;
    int n_blocking;
    double has_cover;
    double close_danger;
    double obstacle_pressure;
    double surprise_desire;
};

struct FuzzyDecision {
    std::string tactic;
    double speed_scale;
    double lookahead_scale;
    bool prefer_alt_path;
    bool use_hiding_obstacle;
    std::map<std::string, double> debug;
};

class FuzzyLogic {
public:
    TacticalFeatures extractFeatures(
        const RobotTrack& me,
        const RobotTrack& enemy,
        const std::vector<Obstacle>& obs);

    FuzzyDecision offense(const TacticalFeatures& f);
    FuzzyDecision defense(const TacticalFeatures& f);
};
