#pragma once
#include "image_transfer.h"
#include "vision.h"
#include "Types.h"
#include <vector>

class Obstacles {
public:
    Obstacles();
    
    // FLOOR MODEL detector (Lab z-score based, lighting adaptive)
    std::vector<Obstacle> detect_floor_model(image& rgb,
                                             image* robot_mask,
                                             float kL, float ka, float kb,
                                             int min_area_param);

    // Wrapper: build robot_mask from Blob list
    std::vector<Obstacle> detect_floor_model(image& rgb,
                                             const std::vector<Blob>& robot_blobs,
                                             float kL, float ka, float kb,
                                             int min_area_param);

private:
    void alloc_grey(image& img, int w, int h);
    std::vector<Obstacle> extract_obstacles(image& binary, image& grey);
};