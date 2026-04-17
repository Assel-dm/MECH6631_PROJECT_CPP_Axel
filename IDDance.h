#pragma once

#include "Types.h"
#include "MarkerDetector.h"
#include "Tracking.h"
#include <vector>
#include <map>

class IDDance {
public:
    IDDance();

    // Run ID dance for a few seconds until identity is determined.
    // Returns my_id (0 or 1 or 2...) or -1 if not found.
    int run(
        double now,
        image& rgb,
        MarkerDetector& detector,
        Tracker& tracker);

    // Whether ID dance is finished
    bool done() const { return done_; }

    // My assigned ID
    int my_id() const { return my_id_; }

    // Get the wheel + laser command for the current dance state
    Command currentCommand() const;

private:
    enum State {
        INIT,
        SPIN,          // The signature dance
        OBSERVE,       // Observe and confirm
        FINISHED
    };

    State state_;
    double state_start_time_;
    int my_id_;
    bool done_;

    // Dance parameters
    double spin_time_;        // Time for full 360° spin
    double observe_time_;     // Time to observe after spin

    // Internal tracking
    std::vector<RobotTrack> tracks_;
    
    // Movement tracking for ID detection
    struct RobotSnapshot {
        double x, y;
        double first_seen;
        double total_movement;  // Accumulated distance traveled
        int sample_count;
    };
    
    std::map<int, RobotSnapshot> robot_snapshots_;  // Track ID -> snapshot
    bool snapshots_initialized_;
};
