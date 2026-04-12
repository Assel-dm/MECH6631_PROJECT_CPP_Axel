#pragma once
#include "Types.h"
#include "image_transfer.h"
#include "vision.h"
#include "Obstacles.h"
#include "OccupancyGrid.h"
#include <vector>

/*
  ObstaclePipeline.h
  Small helper API that builds a robot-exclusion mask, calls Obstacles::detect_floor_model
  and converts results to an occupancy Grid.
*/

struct ObstaclePipelineResult {
    std::vector<Obstacle> obstacles;
    Grid occ_grid;
};

// Process a frame to produce obstacles and an occupancy grid.
// - robot_blobs: centers of markers (front+rear combined) in pixel coords (Blob.x, Blob.y).
// - obst: existing Obstacles instance (reused).
// - grid_builder: existing OccupancyGrid instance.
// Returns obstacles + grid. Uses vision/image_transfer primitives (no OpenCV).
ObstaclePipelineResult process_frame_obstacles(
    image& rgb,
    const std::vector<Blob>& robot_blobs,
    Obstacles& obst,
    OccupancyGrid& grid_builder,
    float kL, float ka, float kb,
    int min_area,
    int cell_px,
    int inflate_px,
    int robot_circle_radius_px = 30,
    int robot_body_width_px = 45);
