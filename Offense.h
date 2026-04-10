#pragma once
#include "Types.h"
#include "Fuzzy.h"
#include "AStar.h"
#include "Waypoint.h"
#include <optional>
#include <vector>

class OffenseStrategy {
public:
    OffenseStrategy(AStarPlanner* planner,
        WaypointFollower* follower,
        FuzzyLogic* fuzzy);

    std::pair<Command, std::optional<std::vector<std::pair<int, int>>>>
        compute(
            const std::vector<RobotTrack>& tracks,
            int my_id,
            const Grid& grid,
            int cell_px,
            int lookahead_cells,
            double laser_close_px,
            double laser_align_deg,
            double v_max,
            const std::vector<Obstacle>& obstacles);

private:
    AStarPlanner* planner_;
    WaypointFollower* follower_;
    FuzzyLogic* fuzzy_;
};
