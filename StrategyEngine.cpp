#include "StrategyEngine.h"
#include <iostream>
#include <cmath>

using namespace std;

StrategyEngine::StrategyEngine()
    : offense_strategy_(&planner_, &follower_, &fuzzy_),
      defense_strategy_(&planner_, &follower_, &fuzzy_)
{
    // Constructor - strategies initialized with dependencies
}

void StrategyEngine::setID(int id) {
    my_id_ = id;
    cout << "Robot ID set to: " << id << endl;
}

void StrategyEngine::setColorProfile(ColorProfile profile) {
    my_profile_ = profile;
    cout << "Robot color profile: ";
    switch (profile) {
        case ColorProfile::BR: cout << "BR (Blue/Red)" << endl; break;
        case ColorProfile::GR: cout << "GR (Green/Red)" << endl; break;
        case ColorProfile::OB: cout << "OB (Orange/Blue)" << endl; break;
    }
}

void StrategyEngine::setOpponentProfile(ColorProfile profile) {
    opp_profile_ = profile;
}

void StrategyEngine::setStrategyMode(StrategyMode mode) {
    strategy_mode_ = mode;
    offense_mode_ = (mode == StrategyMode::OFFENSE);
    
    cout << "Strategy mode: ";
    switch (mode) {
        case StrategyMode::OFFENSE: cout << "OFFENSE" << endl; break;
        case StrategyMode::DEFENSE: cout << "DEFENSE" << endl; break;
        case StrategyMode::AUTO:    cout << "AUTO" << endl; break;
    }
}

void StrategyEngine::setArenaSize(int width, int height) {
    arena_width_ = width;
    arena_height_ = height;
    cout << "Arena size: " << width << "x" << height << " pixels" << endl;
}

void StrategyEngine::setMaxSpeed(double max_speed) {
    max_speed_ = max_speed;
    v_max_ = 1.0;  // Normalized (follower uses 0-1 range)
}

void StrategyEngine::setCellSize(int cell_px) {
    cell_px_ = cell_px;
}

void StrategyEngine::setRobotDimensions(double width, double length) {
    robot_width_ = width;
    robot_length_ = length;
}

// Main update function
Command StrategyEngine::update(image& rgb, double tc) {
    // Detect markers using configured profiles
    std::vector<Blob> front, rear;
    detector_.detect_two_profiles(rgb, my_profile_, opp_profile_, front, rear);
    
    // Update tracking
    std::optional<double> expected_sep;
    auto dets = tracker_.pairMarkers(front, rear, expected_sep, 0.55, 1200.0);
    tracks_ = tracker_.updateTracks(tracks_, dets, tc, 80.0, 10);
    
    // Run strategy based on configured mode
    Command cmd = {0.0, 0.0, false};
    
    switch (strategy_mode_) {
        case StrategyMode::OFFENSE:
            cmd = runOffenseStrategy(rgb, tc);
            break;
        case StrategyMode::DEFENSE:
            cmd = runDefenseStrategy(rgb, tc);
            break;
        case StrategyMode::AUTO:
            cmd = runAdaptiveStrategy(rgb, tc);
            break;
    }
    
    return cmd;
}

// Offense strategy implementation
Command StrategyEngine::runOffenseStrategy(image& rgb, double tc) {
    // TODO: Implement full offense logic
    // For now, return simple forward command for testing
    Command cmd = {0.3, 0.3, false};  // Slow forward
    
    if (tracks_.size() >= 2) {
        // Basic: drive toward opponent
        // TODO: Use actual offense_strategy_.compute()
    }
    
    return cmd;
}

// Defense strategy implementation
Command StrategyEngine::runDefenseStrategy(image& rgb, double tc) {
    // TODO: Implement full defense logic
    // For now, return slow rotation for testing
    Command cmd = {-0.2, 0.2, false};  // Slow turn
    
    if (tracks_.size() >= 2) {
        // Basic: rotate to face opponent
        // TODO: Use actual defense_strategy_.compute()
    }
    
    return cmd;
}

// Adaptive strategy (decides offense/defense based on game state)
Command StrategyEngine::runAdaptiveStrategy(image& rgb, double tc) {
    // Simple heuristic: offense if we see opponent, defense otherwise
    if (tracks_.size() >= 2) {
        return runOffenseStrategy(rgb, tc);
    } else {
        return runDefenseStrategy(rgb, tc);
    }
}