#pragma once
#include "Types.h"
#include "image_transfer.h"
#include "vision.h"
#include "Tracking.h"         
#include "Obstacles.h"
#include "OccupancyGrid.h"
#include "AStar.h"
#include "Waypoint.h"
#include "Fuzzy.h"
#include "Offense.h"
#include "Defense.h"
#include "ColorProfiles.h"
#include "LaserGate.h"

#include <map>
#include <optional>
#include <string>
#include <vector>

class StrategyEngine {
public:
    StrategyEngine();
    void setID(int id);
    Command update(image& rgb, double now);

private:
    // New profile-aware perception members
    std::map<std::string, ColorProfiles::ColorSpec> color_specs_;
    std::map<std::string, ColorProfiles::RobotColorProfile> robot_profiles_;
    std::map<std::string, std::optional<double>> expected_sep_px_map_;
    std::vector<ColorProfiles::ProfiledRobotTrack> prof_tracks_;

    // Legacy consumers (Fuzzy / Offense / Defense) still use RobotTrack for now
    std::vector<RobotTrack> tracks_;

    Obstacles obstacleDetector_;
    OccupancyGrid occBuilder_;
    AStarPlanner planner_;
    WaypointFollower follower_;
    FuzzyLogic fuzzy_;
    OffenseStrategy offense_;
    DefenseStrategy defense_;
    LaserGate laser_gate_;

    int my_id_;
    bool offense_mode_;

    // Tunable parameters ------------------------------------------------------
    int cell_px_;
    int inflate_px_;
    int lookahead_cells_;
    double v_max_;
    double laser_close_px_;
    double laser_align_deg_;
    double laser_los_margin_px_;
    double max_match_dist_px_;
    int max_misses_;

    // Marker / profile tuning
    int blob_min_area_;
    int blob_max_area_;
    double min_area_ratio_;
    double pair_sep_tol_;
    double max_pair_px_;
    double pair_area_ratio_tol_;
    int morph_open_iters_;
    int morph_close_iters_;
    int morph_repeat_;

    // Lab floor model thresholds (lighting adaptive)
    float kL_;
    float ka_;
    float kb_;

    // Arena boundary enforcement ---------------------------------------------
    int arena_margin_px_;
    int arena_danger_px_;
    void addArenaBoundaries(std::vector<Obstacle>& obs, int W, int H) const;

    // Helpers -----------------------------------------------------------------
    static std::vector<RobotDet> toLegacyDets(const std::vector<ColorProfiles::ProfiledRobotDet>& dets);
    static std::vector<RobotTrack> toLegacyTracks(const std::vector<ColorProfiles::ProfiledRobotTrack>& tracks);
};
