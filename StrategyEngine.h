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

    // Lab floor model thresholds (lighting adaptive)
    float kL_;
    float ka_;
    float kb_;

    // ── Arena boundary enforcement ────────────────────────────────────────────
    // arena_margin_px_: width (px) of the virtual wall strip along each camera
    //   edge added to the obstacle list so A* never plans into the border zone.
    //   Increase if the robot still approaches the edge during normal play.
    int arena_margin_px_;

    // arena_danger_px_: if the robot's tracked centroid is within this many px
    //   of any frame edge, all motion is halted regardless of A* output.
    //   Set >= arena_margin_px_ so the hard-stop only fires when the path guard
    //   has already failed (e.g. tracking lag or a sudden shove).
    int arena_danger_px_;

    // Appends four thin Obstacle strips (top/bottom/left/right) to `obs`.
    void addArenaBoundaries(std::vector<Obstacle>& obs, int W, int H) const;
};