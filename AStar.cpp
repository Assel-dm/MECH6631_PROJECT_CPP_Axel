#include "AStar.h"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <optional>

// ------------------------------
// Node key + hashing
// ------------------------------
struct NodeKey {
    int y, x;
    bool operator==(const NodeKey& o) const { return y == o.y && x == o.x; }
};

struct NodeKeyHash {
    std::size_t operator()(const NodeKey& k) const {
        return (std::hash<int>()(k.y) << 1) ^ std::hash<int>()(k.x);
    }
};

struct PQItem {
    double f;
    NodeKey key;
};

struct PQCompare {
    bool operator()(const PQItem& a, const PQItem& b) const {
        return a.f > b.f;   // min-heap
    }
};

// ------------------------------
// A* implementation
// ------------------------------
std::optional<std::vector<std::pair<int, int>>> AStarPlanner::plan(
    const Grid& grid,
    std::pair<int, int> start,
    std::pair<int, int> goal)
{
    int gh = grid.size();
    if (gh == 0) return std::nullopt;
    int gw = grid[0].size();

    auto in_bounds = [&](int y, int x) {
        return (y >= 0 && y < gh && x >= 0 && x < gw);
        };

    int sy = start.first, sx = start.second;
    int gy = goal.first, gx = goal.second;

    if (!in_bounds(sy, sx) || !in_bounds(gy, gx)) return std::nullopt;
    if (grid[gy][gx]) return std::nullopt; 

    std::vector<std::pair<int, int>> nbrs = {
        {-1,0},{1,0},{0,-1},{0,1},
        {-1,-1},{-1,1},{1,-1},{1,1}
    };

    auto h_cost = [&](int y, int x) {
        return std::hypot((double)(y - gy), (double)(x - gx));
        };

    std::priority_queue<PQItem, std::vector<PQItem>, PQCompare> open;

    std::unordered_map<NodeKey, NodeKey, NodeKeyHash> came_from;
    std::unordered_map<NodeKey, double, NodeKeyHash> gscore;

    NodeKey s{ sy, sx };
    NodeKey g{ gy, gx };

    gscore[s] = 0.0;
    open.push(PQItem{ h_cost(sy, sx), s });

    while (!open.empty()) {
        PQItem item = open.top();
        open.pop();

        NodeKey cur = item.key;

        if (cur.y == g.y && cur.x == g.x) {
            std::vector<std::pair<int, int>> path;
            NodeKey c = cur;
            path.push_back({ c.y, c.x });

            while (came_from.find(c) != came_from.end()) {
                c = came_from[c];
                path.push_back({ c.y, c.x });
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        for (auto& nb : nbrs) {
            int ny = cur.y + nb.first;
            int nx = cur.x + nb.second;
            if (!in_bounds(ny, nx) || grid[ny][nx] == 1) continue;

            double step = std::hypot((double)nb.first, (double)nb.second);
            double tentative = gscore[cur] + step;

            NodeKey nk{ ny, nx };
            auto it = gscore.find(nk);

            if (it == gscore.end() || tentative < it->second) {
                gscore[nk] = tentative;
                came_from[nk] = cur;
                open.push(PQItem{ tentative + h_cost(ny, nx), nk });
            }
        }
    }

    return std::nullopt;
}
