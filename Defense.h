#pragma once
#include "Types.h"
#include "Fuzzy.h"
#include "AStar.h"
#include "Waypoint.h"
#include <optional>
#include <vector>

// DefenseStrategy — hides our robot behind obstacles to break line-of-sight
// from the enemy laser. Uses A* path planning and fuzzy-scaled speed control.
class DefenseStrategy {
public:
    struct Result {
        Command cmd;
        std::optional<std::vector<std::pair<int, int>>> path;
        FuzzyDecision decision;
        std::optional<std::pair<double, double>> hiding_point;
    };

    DefenseStrategy(AStarPlanner* planner,
        WaypointFollower* follower,
        FuzzyLogic* fuzzy);

    Result compute(
        const std::vector<RobotTrack>& tracks,
        int my_id,
        const Grid& grid,           // Inflated grid — used for path planning
        const Grid& grid_visual,    // Uninflated grid — used for LOS checks
        int cell_px,
        int lookahead_cells,
        int samples,
        double v_max,
        double robot_half_length_px, // Half robot body length: prevents clipping obstacle edges
        const std::vector<Obstacle>& obstacles);

private:
    AStarPlanner*    planner_;
    WaypointFollower* follower_;
    FuzzyLogic*      fuzzy_;

    // Persisted hiding goal so the robot doesn't thrash between frames
    std::optional<std::pair<double, double>> cached_goal_;
};