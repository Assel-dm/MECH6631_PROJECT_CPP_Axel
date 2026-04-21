#pragma once
#include <iostream>

// Test modes
enum TestMode {
    TEST_ID_DANCE,
    TEST_OFFENSE,
    TEST_DEFENSE,
    TEST_NAVIGATION,
    TEST_FULL_GAME
};

// Populate obstacle arrays based on test mode and arena scale
void setupObstacles(TestMode mode, int& N_obs,
                    double x_obs[], double y_obs[], double size_obs[],
                    double scale = 1.0);

// Append four boundary wall obstacles to keep robots inside the arena
void addBoundaryObstacles(int& N_obs,
                          double x_obs[], double y_obs[], double size_obs[],
                          double width, double height, double margin = 40.0);
