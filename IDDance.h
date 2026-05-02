#pragma once

#include "Types.h"
#include "MarkerDetector.h"
#include "Tracking.h"
#include <vector>
#include <map>

class IDDance {
public:
    IDDance();

    int run(
        double now,
        image& rgb,
        MarkerDetector& detector,
        Tracker& tracker);

    bool done() const { return done_; }
    int my_id() const { return my_id_; }
    Command currentCommand() const;

private:
    enum State {
        INIT,
        SPIN,
        OBSERVE,
        FINISHED
    };

    State state_;
    double state_start_time_;
    int my_id_;
    bool done_;

    double spin_time_;
    double observe_time_;

    std::vector<RobotTrack> tracks_;

    struct RobotSnapshot {
        double x, y;
        double theta;
        double first_seen;
        double total_movement;
        int sample_count;

        // Angular velocity tracking
        double prev_theta;
        double prev_time;
        double total_dtheta;    // Accumulated |angular change|
        int    omega_samples;
    };

    std::map<int, RobotSnapshot> robot_snapshots_;
    bool snapshots_initialized_;

    // Commanded spin angular velocity (rad/s) — calibrated from spin_time_
    // A full 360° spin in spin_time_ seconds → omega = 2π / spin_time_
    double commanded_omega_;
};
