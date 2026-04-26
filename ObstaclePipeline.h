#pragma once
#include "Types.h"
#include "image_transfer.h"
#include "vision.h"
#include "Obstacles.h"
#include "OccupancyGrid.h"
#include <vector>
#include "MarkerDetector.h"

struct ObstaclePipelineResult {
    std::vector<Obstacle> obstacles;
    Grid occ_grid;
};

// Process frame with oriented robot exclusion rectangles
// - robot_dets: paired robot detections (front+rear markers with pose)
// - robot_length_px: length of robot body (front to rear, along heading)
// - robot_width_px: width of robot body (perpendicular to heading)
ObstaclePipelineResult process_frame_obstacles(
    image& rgb,
    const std::vector<RobotDet>& robot_dets,  // CHANGED: use paired detections
    Obstacles& obst,
    OccupancyGrid& grid_builder,
    float kL, float ka, float kb,    // <-- RESTORE THESE (not gray_threshold)
    int min_area,
    int cell_px,
    int inflate_px,
    int robot_length_px = 60,    // NEW: adjustable length
    int robot_width_px = 40);    // NEW: adjustable width
