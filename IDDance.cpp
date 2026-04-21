#include "IDDance.h"
#include <cmath>
#include <iostream>

// Helper: wrap angle to [-π, π]
static double wrap_angle(double a) {
    return fmod(a + M_PI, 2.0 * M_PI) - M_PI;
}

IDDance::IDDance()
{
    state_ = INIT;
    state_start_time_ = 0.0;
    my_id_ = -1;
    done_ = false;
    snapshots_initialized_ = false;

	spin_time_ = 4.7; //This should be properly calibrated based on physical testing to achieve a clear angular velocity signature
    observe_time_ = 1.0;

    // Expected angular velocity during spin (both wheels -0.8 → spinning in place)
    // One full revolution in spin_time_ seconds
    commanded_omega_ = (2.0 * M_PI) / spin_time_;   // ~1.337 rad/s
}

Command IDDance::currentCommand() const
{
    Command cmd{ 0.0, 0.0, false };

    switch (state_)
    {
    case SPIN:
        cmd.left  = -0.8;
        cmd.right = -0.8;
        cmd.laser = false;
        break;
    case OBSERVE:
        cmd.left  = 0.0;
        cmd.right = 0.0;
        cmd.laser = false;
        break;
    default:
        cmd.left  = 0.0;
        cmd.right = 0.0;
        cmd.laser = false;
        break;
    }

    return cmd;
}

int IDDance::run(
    double now,
    image& rgb,
    MarkerDetector& detector,
    Tracker& tracker)
{
    if (done_) return my_id_;

    std::vector<Blob> front, rear;
    detector.detect_markers(rgb, front, rear);

    std::optional<double> expected_sep;
    auto dets = tracker.pairMarkers(front, rear, expected_sep, 0.55, 1200.0);

    tracks_ = tracker.updateTracks(tracks_, dets, now, 80.0, 10);

    switch (state_) {

    case INIT:
        state_start_time_ = now;
        state_ = SPIN;
        snapshots_initialized_ = false;
        robot_snapshots_.clear();
        std::cout << "Starting ID dance - will spin 360 degrees" << std::endl;
        std::cout << "  Expected omega = " << commanded_omega_ << " rad/s" << std::endl;
        break;

    case SPIN:
    {
        // Initialise snapshots on first frame
        if (!snapshots_initialized_ && !tracks_.empty()) {
            for (const auto& t : tracks_) {
                RobotSnapshot snap;
                snap.x = t.x;
                snap.y = t.y;
                snap.theta        = t.theta;
                snap.first_seen   = now;
                snap.total_movement = 0.0;
                snap.sample_count = 0;
                snap.prev_theta   = t.theta;
                snap.prev_time    = now;
                snap.total_dtheta = 0.0;
                snap.omega_samples = 0;

                robot_snapshots_[t.id] = snap;
                std::cout << "  Tracking robot " << t.id
                          << " at (" << t.x << ", " << t.y << ")"
                          << " theta=" << t.theta << std::endl;
            }
            snapshots_initialized_ = true;
        }

        // Accumulate translational movement AND angular velocity each frame
        if (snapshots_initialized_) {
            for (const auto& t : tracks_) {
                auto it = robot_snapshots_.find(t.id);
                if (it == robot_snapshots_.end()) continue;

                auto& snap = it->second;

                // Translation
                double dx   = t.x - snap.x;
                double dy   = t.y - snap.y;
                snap.total_movement += std::hypot(dx, dy);
                snap.sample_count++;
                snap.x = t.x;
                snap.y = t.y;

                // Angular velocity
                double dt = now - snap.prev_time;
                if (dt > 1e-4) {
                    double dtheta = wrap_angle(t.theta - snap.prev_theta);
                    double omega  = dtheta / dt;           // rad/s (signed)
                    snap.total_dtheta  += std::fabs(omega); // accumulate magnitude
                    snap.omega_samples++;
                }
                snap.prev_theta = t.theta;
                snap.prev_time  = now;
                snap.theta      = t.theta;
            }
        }

        if (now - state_start_time_ > spin_time_) {
            state_start_time_ = now;
            state_ = OBSERVE;
            std::cout << "\nSpin complete! Analyzing angular velocity..." << std::endl;

            for (const auto& pair : robot_snapshots_) {
                double avg_omega = (pair.second.omega_samples > 0)
                    ? pair.second.total_dtheta / pair.second.omega_samples
                    : 0.0;
                std::cout << "  Robot " << pair.first
                          << ": avg_omega=" << avg_omega << " rad/s"
                          << " (target=" << commanded_omega_ << ")"
                          << " movement=" << pair.second.total_movement << " px"
                          << " over " << pair.second.sample_count << " samples"
                          << std::endl;
            }
        }
        break;
    }

    case OBSERVE:
        if (now - state_start_time_ > observe_time_) {

            if (!robot_snapshots_.empty()) {
                int    best_id    = -1;
                double best_score = 1e9;   // Lower = closer to commanded_omega_

                for (const auto& pair : robot_snapshots_) {
                    if (pair.second.omega_samples < 10) continue;

                    double avg_omega = pair.second.total_dtheta / pair.second.omega_samples;
                    double score     = std::fabs(avg_omega - commanded_omega_);

                    if (score < best_score) {
                        best_score = score;
                        best_id    = pair.first;
                    }
                }

                if (best_id >= 0) {
                    my_id_ = best_id;
                    double avg_omega = robot_snapshots_[best_id].total_dtheta
                                     / robot_snapshots_[best_id].omega_samples;
                    std::cout << "\n✅ Identified self as robot ID: " << my_id_
                              << " (omega=" << avg_omega << " rad/s"
                              << ", error=" << std::fabs(avg_omega - commanded_omega_) << " rad/s)"
                              << std::endl;
                } else {
                    // Fallback: use movement (original method)
                    my_id_ = -1;
                    double max_movement = -1.0;
                    for (const auto& pair : robot_snapshots_) {
                        if (pair.second.total_movement > max_movement) {
                            max_movement = pair.second.total_movement;
                            my_id_       = pair.first;
                        }
                    }
                    std::cout << "\n⚠️ Insufficient omega samples, fell back to movement ID: "
                              << my_id_ << std::endl;
                }
            } else {
                my_id_ = (!tracks_.empty()) ? tracks_[0].id : 0;
                std::cout << "\n⚠️ No snapshot data, using first track ID: " << my_id_ << std::endl;
            }

            done_  = true;
            state_ = FINISHED;
            return my_id_;
        }
        break;

    case FINISHED:
        return my_id_;
    }

    return my_id_;
}