#define _USE_MATH_DEFINES
#include "MarkerPairing.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

using namespace std;

RobotDet compute_pose(const Blob& front, const Blob& rear)
{
    double fx = front.x, fy = front.y;
    double rx = rear.x, ry = rear.y;
    double dx = fx - rx;
    double dy = fy - ry;
    double sep = std::hypot(dx, dy);
    double th = std::atan2(dy, dx);
    double mx = 0.5 * (fx + rx);
    double my = 0.5 * (fy + ry);

    RobotDet rd;
    rd.front = front;
    rd.rear = rear;
    rd.x = mx;
    rd.y = my;
    rd.theta = th;
    rd.sep_px = sep;
    return rd;
}

std::optional<double> estimate_marker_sep_px(const std::vector<Blob>& front_blobs,
                                             const std::vector<Blob>& rear_blobs)
{
    if (front_blobs.empty() || rear_blobs.empty()) return std::nullopt;

    std::vector<double> dists;
    dists.reserve(front_blobs.size());
    for (const auto &f : front_blobs) {
        double fx = f.x, fy = f.y;
        double best = std::numeric_limits<double>::infinity();
        for (const auto &r : rear_blobs) {
            double d = std::hypot(fx - r.x, fy - r.y);
            if (d < best) best = d;
        }
        if (std::isfinite(best)) dists.push_back(best);
    }
    if (dists.empty()) return std::nullopt;
    std::sort(dists.begin(), dists.end());
    size_t n = dists.size();
    if (n % 2 == 1) return dists[n/2];
    return 0.5 * (dists[n/2 - 1] + dists[n/2]);
}

std::vector<RobotDet> pair_markers(const std::vector<Blob>& front_blobs,
                                   const std::vector<Blob>& rear_blobs,
                                   std::optional<double> expected_sep_px,
                                   double sep_tol,
                                   double max_pair_px)
{
    std::vector<RobotDet> dets;
    if (front_blobs.empty() || rear_blobs.empty()) return dets;

    // 1) If expected separation known -> distance gating + greedy best-match
    if (expected_sep_px.has_value()) {
        double expected = expected_sep_px.value();
        double dmin = (1.0 - sep_tol) * expected;
        double dmax = (1.0 + sep_tol) * expected;

        // candidates: (abs(error), i_front, j_rear)
        std::vector<std::tuple<double, size_t, size_t>> cands;
        for (size_t i = 0; i < front_blobs.size(); ++i) {
            double fx = front_blobs[i].x, fy = front_blobs[i].y;
            for (size_t j = 0; j < rear_blobs.size(); ++j) {
                double rx = rear_blobs[j].x, ry = rear_blobs[j].y;
                double d = std::hypot(fx - rx, fy - ry);
                if (d >= dmin && d <= dmax) {
                    cands.emplace_back(std::abs(d - expected), i, j);
                }
            }
        }
        std::sort(cands.begin(), cands.end(), [](const auto &a, const auto &b){
            return std::get<0>(a) < std::get<0>(b);
        });

        std::vector<char> used_f(front_blobs.size(), 0), used_r(rear_blobs.size(), 0);
        for (const auto &t : cands) {
            size_t i = std::get<1>(t);
            size_t j = std::get<2>(t);
            if (used_f[i] || used_r[j]) continue;
            used_f[i] = used_r[j] = 1;
            dets.push_back(compute_pose(front_blobs[i], rear_blobs[j]));
        }
        return dets;
    }

    // 2) Fallback: mutual nearest-neighbor pairing (bring-up)
    // front -> nearest rear
    std::vector<std::pair<int,double>> f2r(front_blobs.size(), {-1, std::numeric_limits<double>::infinity()});
    for (size_t i = 0; i < front_blobs.size(); ++i) {
        double fx = front_blobs[i].x, fy = front_blobs[i].y;
        int best_j = -1;
        double best_d = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < rear_blobs.size(); ++j) {
            double rx = rear_blobs[j].x, ry = rear_blobs[j].y;
            double d = std::hypot(fx - rx, fy - ry);
            if (d < best_d) { best_d = d; best_j = (int)j; }
        }
        f2r[i] = {best_j, best_d};
    }

    // rear -> nearest front
    std::vector<std::pair<int,double>> r2f(rear_blobs.size(), {-1, std::numeric_limits<double>::infinity()});
    for (size_t j = 0; j < rear_blobs.size(); ++j) {
        double rx = rear_blobs[j].x, ry = rear_blobs[j].y;
        int best_i = -1;
        double best_d = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < front_blobs.size(); ++i) {
            double fx = front_blobs[i].x, fy = front_blobs[i].y;
            double d = std::hypot(fx - rx, fy - ry);
            if (d < best_d) { best_d = d; best_i = (int)i; }
        }
        r2f[j] = {best_i, best_d};
    }

    std::vector<char> used_front(front_blobs.size(), 0), used_rear(rear_blobs.size(), 0);
    for (size_t i = 0; i < front_blobs.size(); ++i) {
        int j = f2r[i].first;
        double d = f2r[i].second;
        if (j < 0) continue;
        if (d > max_pair_px) continue;
        int i2 = r2f[j].first;
        if (i2 != (int)i) continue; // not mutual
        if (used_front[i] || used_rear[j]) continue;
        used_front[i] = used_rear[j] = 1;
        dets.push_back(compute_pose(front_blobs[i], rear_blobs[j]));
    }

    return dets;
}