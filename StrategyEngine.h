#pragma once
#include "Types.h"
#include "image_transfer.h"
#include "vision.h"
#include "MarkerDetector.h"
#include "Tracking.h"
#include "Obstacles.h"
#include "OccupancyGrid.h"
#include "AStar.h"
#include "Waypoint.h"
#include "Fuzzy.h"
#include "Offense.h"
#include "Defense.h"

class StrategyEngine {
public:
    StrategyEngine();
    void setID(int id);
    Command update(image& rgb, double now);

private:
    MarkerDetector markerDetector_;
    Tracker tracker_;
    Obstacles obstacleDetector_;
    OccupancyGrid occBuilder_;
    AStarPlanner planner_;
    WaypointFollower follower_;
    FuzzyLogic fuzzy_;
    OffenseStrategy offense_;
    DefenseStrategy defense_;

    std::vector<RobotTrack> tracks_;
    int my_id_;
    bool offense_mode_;

    // Tunable parameters
    int cell_px_;
    int inflate_px_;
    int lookahead_cells_;
    double v_max_;
    double laser_close_px_;
    double laser_align_deg_;
    double max_match_dist_px_;
    int max_misses_;

};