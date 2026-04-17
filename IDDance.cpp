#include "IDDance.h"
#include <cmath>
#include <iostream>

IDDance::IDDance()
{
    state_ = INIT;
    state_start_time_ = 0.0;
    my_id_ = -1;
    done_ = false;
    snapshots_initialized_ = false;

    spin_time_ = 4.7;        // Time for full 360° spin
    observe_time_ = 1.0;     // Shorter observe time - we already know who danced
}

Command IDDance::currentCommand() const
{
    Command cmd{ 0.0, 0.0, false };

    switch (state_)
    {
    case SPIN:
        // The signature dance: 360° spin
        cmd.left = -0.8;
        cmd.right = -0.8;
        cmd.laser = false;
        break;

    case OBSERVE:
        // Stationary while observing
        cmd.left = 0.0;
        cmd.right = 0.0;
        cmd.laser = false;
        break;

    default:
        // INIT and FINISHED states
        cmd.left = 0.0;
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

    // Detect markers
    std::vector<Blob> front, rear;
    detector.detect_markers(rgb, front, rear);

    // Pair into robot detections
    std::optional<double> expected_sep;
    auto dets = tracker.pairMarkers(front, rear, expected_sep, 0.55, 1200.0);

    // Update tracks
    tracks_ = tracker.updateTracks(
        tracks_,
        dets,
        now,
        80.0,
        10
    );

    // -------------------------
    // State machine
    // -------------------------

    switch (state_) {

    case INIT:
        state_start_time_ = now;
        state_ = SPIN;
        snapshots_initialized_ = false;
        robot_snapshots_.clear();
        std::cout << "Starting ID dance - will spin 360 degrees" << std::endl;
        break;

    case SPIN:
    {
        // Initialize snapshots on first frame of spin
        if (!snapshots_initialized_ && !tracks_.empty()) {
            for (const auto& t : tracks_) {
                RobotSnapshot snap;
                snap.x = t.x;
                snap.y = t.y;
                snap.first_seen = now;
                snap.total_movement = 0.0;
                snap.sample_count = 0;
                robot_snapshots_[t.id] = snap;
                
                std::cout << "  Tracking robot " << t.id 
                         << " at (" << t.x << ", " << t.y << ")" << std::endl;
            }
            snapshots_initialized_ = true;
        }

        // Track movement during spin
        if (snapshots_initialized_) {
            for (const auto& t : tracks_) {
                auto it = robot_snapshots_.find(t.id);
                if (it != robot_snapshots_.end()) {
                    // Calculate movement since last frame
                    double dx = t.x - it->second.x;
                    double dy = t.y - it->second.y;
                    double dist = std::hypot(dx, dy);
                    
                    // Accumulate total movement
                    it->second.total_movement += dist;
                    it->second.sample_count++;
                    
                    // Update position for next frame
                    it->second.x = t.x;
                    it->second.y = t.y;
                }
            }
        }

        // Check if spin complete
        if (now - state_start_time_ > spin_time_) {
            state_start_time_ = now;
            state_ = OBSERVE;
            std::cout << "\nSpin complete! Analyzing movement..." << std::endl;
            
            // Debug: Print movement statistics
            for (const auto& pair : robot_snapshots_) {
                std::cout << "  Robot " << pair.first 
                         << ": total_movement=" << pair.second.total_movement 
                         << " px over " << pair.second.sample_count << " samples" 
                         << std::endl;
            }
        }
        break;
    }

    case OBSERVE:
        if (now - state_start_time_ > observe_time_) {

            // Identify robot with MAXIMUM movement as "me" (the dancer)
            if (!robot_snapshots_.empty()) {
                int best_id = -1;
                double max_movement = -1.0;
                
                for (const auto& pair : robot_snapshots_) {
                    // Only consider robots with enough samples
                    if (pair.second.sample_count > 10) {
                        if (pair.second.total_movement > max_movement) {
                            max_movement = pair.second.total_movement;
                            best_id = pair.first;
                        }
                    }
                }
                
                if (best_id >= 0) {
                    my_id_ = best_id;
                    std::cout << "\n✅ Identified self as robot ID: " << my_id_ 
                             << " (moved " << max_movement << " pixels)" << std::endl;
                } else {
                    // Fallback: use first tracked robot
                    my_id_ = robot_snapshots_.begin()->first;
                    std::cout << "\n⚠️ Insufficient samples, using first robot ID: " 
                             << my_id_ << std::endl;
                }
            } else {
                // No snapshots - fallback to current tracks
                if (!tracks_.empty()) {
                    my_id_ = tracks_[0].id;
                    std::cout << "\n⚠️ No movement data, using first track ID: " 
                             << my_id_ << std::endl;
                } else {
                    my_id_ = 0;
                    std::cout << "\n⚠️ No tracks detected, assigning default ID: 0" 
                             << std::endl;
                }
            }

            done_ = true;
            state_ = FINISHED;
            
            return my_id_;
        }
        break;

    case FINISHED:
        return my_id_;
    }

    return my_id_;
}