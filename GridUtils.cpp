#include "GridUtils.h"

#include <queue>
#include <cmath>

namespace GridUtils {

std::pair<int,int> pixToCell(double x, double y, int cell_px)
{
    return { static_cast<int>(y / cell_px), static_cast<int>(x / cell_px) };
}

std::pair<double,double> cellToPix(int row, int col, int cell_px)
{
    return { (col + 0.5) * cell_px, (row + 0.5) * cell_px };
}

bool inBounds(const Grid& grid, int row, int col)
{
    if (grid.empty()) return false;
    return row >= 0 && row < static_cast<int>(grid.size()) &&
           col >= 0 && col < static_cast<int>(grid[0].size());
}

bool isBlocked(const Grid& grid, int row, int col)
{
    if (!inBounds(grid, row, col)) return true;
    return grid[row][col] != 0;
}

std::optional<std::pair<int,int>> snapToNearestFree(
    const Grid& grid,
    std::pair<int,int> start_cell,
    int max_radius_cells)
{
    const int sr = start_cell.first;
    const int sc = start_cell.second;

    if (!inBounds(grid, sr, sc)) return std::nullopt;
    if (!isBlocked(grid, sr, sc)) return start_cell;

    std::queue<std::pair<int,int>> q;
    std::vector<std::vector<int>> dist(
        grid.size(),
        std::vector<int>(grid[0].size(), -1));

    dist[sr][sc] = 0;
    q.push({sr, sc});

    const int dr[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dc[8] = { 0, 0,-1, 1, -1,  1,-1, 1};

    while (!q.empty()) {
        auto cur = q.front();
        q.pop();

        const int r = cur.first;
        const int c = cur.second;
        const int d = dist[r][c];

        if (d > max_radius_cells) continue;

        if (!isBlocked(grid, r, c)) {
            return cur;
        }

        for (int k = 0; k < 8; ++k) {
            const int nr = r + dr[k];
            const int nc = c + dc[k];
            if (!inBounds(grid, nr, nc)) continue;
            if (dist[nr][nc] != -1) continue;
            dist[nr][nc] = d + 1;
            q.push({nr, nc});
        }
    }

    return std::nullopt;
}

std::optional<std::pair<int,int>> safePixToFreeCell(
    const Grid& grid,
    double x,
    double y,
    int cell_px,
    int max_radius_cells)
{
    auto cell = pixToCell(x, y, cell_px);
    return snapToNearestFree(grid, cell, max_radius_cells);
}

std::vector<std::pair<double,double>> pathCellsToPixels(
    const std::vector<std::pair<int,int>>& path,
    int cell_px)
{
    std::vector<std::pair<double,double>> out;
    out.reserve(path.size());
    for (const auto& rc : path) {
        out.push_back(cellToPix(rc.first, rc.second, cell_px));
    }
    return out;
}

} // namespace GridUtils
