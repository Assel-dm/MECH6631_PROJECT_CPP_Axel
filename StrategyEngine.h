#pragma once
#include "Types.h"
#include "MarkerDetector.h"
#include "Tracking.h"
#include "AStar.h"
#include "OccupancyGrid.h"
#include "Waypoint.h"
#include "Fuzzy.h"
#include "Offense.h"
#include "Defense.h"
#include <vector>

class StrategyEngine {
public:
    StrategyEngine();
    
    // Configuration methods
    void setID(int id);
    void setColorProfile(ColorProfile profile);
    void setOpponentProfile(ColorProfile profile);
    void setStrategyMode(StrategyMode mode);
    void setArenaSize(int width, int height);
    void setMaxSpeed(double max_speed);
    void setCellSize(int cell_px);
    void setRobotDimensions(double width, double length);
    
    // Main update function
    Command update(image& rgb, double tc);
    
private:
    // Strategy helper functions
    Command runOffenseStrategy(image& rgb, double tc);
    Command runDefenseStrategy(image& rgb, double tc);
    Command runAdaptiveStrategy(image& rgb, double tc);
    
    // Configuration members
    int my_id_ = -1;
    ColorProfile my_profile_ = ColorProfile::BR;
    ColorProfile opp_profile_ = ColorProfile::GR;
    StrategyMode strategy_mode_ = StrategyMode::OFFENSE;
    
    int arena_width_ = 1920;
    int arena_height_ = 1080;
    double max_speed_ = 100.0;
    double v_max_ = 1.0;  // ⭐ Normalized velocity max
    int cell_px_ = 20;
    double robot_width_ = 90.0;
    double robot_length_ = 140.0;
    
    bool offense_mode_ = true;  // ⭐ Derived from strategy_mode_
    
    // Vision and tracking
    MarkerDetector detector_;
    Tracker tracker_;
    std::vector<RobotTrack> tracks_;
    
    // Strategy components
    AStarPlanner planner_;
    WaypointFollower follower_;
    FuzzyLogic fuzzy_;
    OccupancyGrid occupancy_grid_;
    OffenseStrategy offense_strategy_;
    DefenseStrategy defense_strategy_;
};