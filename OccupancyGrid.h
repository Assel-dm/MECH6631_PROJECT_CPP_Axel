#pragma once
#include "Types.h"
#include "image_transfer.h"
#include "vision.h"
#include <vector>

class OccupancyGrid {
public:
    OccupancyGrid();

    // Build occupancy grid from obstacle bounding boxes
    Grid build(
        const std::vector<Obstacle>& obs,
        int width,
        int height,
        int cell_px,
        int inflate_px
    );

private:
    // Build a binary mask of obstacles
    void build_mask(
        const std::vector<Obstacle>& obs,
        image& mask
    );

    // Inflate obstacles using professor's dialate2
    void inflate_mask(
        image& mask,
        int inflate_px
    );

    // Convert inflated mask to occupancy grid
    Grid mask_to_grid(
        image& mask,
        int cell_px
    );

    // Utility: allocate GREY image
    void alloc_grey(image& img, int w, int h);
};
