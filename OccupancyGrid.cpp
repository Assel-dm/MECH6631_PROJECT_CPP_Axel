#include "OccupancyGrid.h"
#include <cstring>
#include <algorithm>

OccupancyGrid::OccupancyGrid() {}

void OccupancyGrid::alloc_grey(image& img, int w, int h) {
    img.type = GREY_IMAGE;
    img.width = w;
    img.height = h;
    allocate_image(img);
}

void OccupancyGrid::build_mask(
    const std::vector<Obstacle>& obs,
    image& mask)
{
    int W = mask.width;
    int H = mask.height;

    // Clear mask
    memset(mask.pdata, 0, W * H);

    // Mark obstacle bounding boxes
    for (const auto& o : obs) {
        int x0 = std::max(0, o.x);
        int y0 = std::max(0, o.y);
        int x1 = std::min(W - 1, o.x + o.w - 1);
        int y1 = std::min(H - 1, o.y + o.h - 1);

        for (int j = y0; j <= y1; j++) {
            for (int i = x0; i <= x1; i++) {
                mask.pdata[j * W + i] = 255;
            }
        }
    }
}

void OccupancyGrid::inflate_mask(image& mask, int inflate_px)
{
    if (inflate_px <= 0) return;

    int W = mask.width;
    int H = mask.height;

    image tmp;
    alloc_grey(tmp, W, H);

    // Perform inflation by repeated dialate2
    for (int k = 0; k < inflate_px; k++) {
        dialate2(mask, tmp);
        copy(tmp, mask);
    }

    free_image(tmp);
}

Grid OccupancyGrid::mask_to_grid(image& mask, int cell_px)
{
    int W = mask.width;
    int H = mask.height;

    int gw = (W + cell_px - 1) / cell_px;
    int gh = (H + cell_px - 1) / cell_px;

    Grid grid(gh, std::vector<uint8_t>(gw, 0));

    for (int gy = 0; gy < gh; gy++) {
        int y0 = gy * cell_px;
        int y1 = std::min(H, (gy + 1) * cell_px);

        for (int gx = 0; gx < gw; gx++) {
            int x0 = gx * cell_px;
            int x1 = std::min(W, (gx + 1) * cell_px);

            bool occ = false;
            for (int j = y0; j < y1 && !occ; j++) {
                for (int i = x0; i < x1; i++) {
                    if (mask.pdata[j * W + i] > 0) {
                        occ = true;
                        break;
                    }
                }
            }

            grid[gy][gx] = occ ? 1 : 0;
        }
    }

    return grid;
}

Grid OccupancyGrid::build(
    const std::vector<Obstacle>& obs,
    int width,
    int height,
    int cell_px,
    int inflate_px)
{
    image mask;
    alloc_grey(mask, width, height);

    // 1. Build raw obstacle mask
    build_mask(obs, mask);

    // 2. Inflate obstacles
    inflate_mask(mask, inflate_px);

    // 3. Convert to occupancy grid
    Grid grid = mask_to_grid(mask, cell_px);

    free_image(mask);
    return grid;
}