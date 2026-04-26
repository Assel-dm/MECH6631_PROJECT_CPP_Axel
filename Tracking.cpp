#include "Tracking.h"
#include <cmath>
#include <algorithm>
#include <limits>

static RobotDet compute_pose(const Blob& f, const Blob& r) {
    double dx = f.x - r.x;
    double dy = f.y - r.y;
    double sep = std::hypot(dx, dy);
    double th = std::atan2(dy, dx);
    double mx = 0.5 * (f.x + r.x);
    double my = 0.5 * (f.y + r.y);
    return RobotDet{ f, r, mx, my, th, sep };
}

std::vector<RobotDet> Tracker::pairMarkers(
    const std::vector<Blob>& front,
    const std::vector<Blob>& rear,
    std::optional<double> expected_sep,
    double sep_tol,
    double max_pair_px)
{
    std::vector<RobotDet> out;
    if (front.empty() || rear.empty()) return out;

    if (expected_sep.has_value()) {
        double dmin = (1.0 - sep_tol) * expected_sep.value();
        double dmax = (1.0 + sep_tol) * expected_sep.value();

        struct Cand { double cost; int fi; int ri; };
        std::vector<Cand> cands;

        for (int i = 0; i < (int)front.size(); i++) {
            for (int j = 0; j < (int)rear.size(); j++) {
                double d = std::hypot(front[i].x - rear[j].x,
                    front[i].y - rear[j].y);
                if (d >= dmin && d <= dmax) {
                    cands.push_back({ std::fabs(d - expected_sep.value()), i, j });
                }
            }
        }

        std::sort(cands.begin(), cands.end(),
            [](auto& a, auto& b) { return a.cost < b.cost; });

        std::vector<bool> used_f(front.size(), false), used_r(rear.size(), false);
        for (auto& c : cands) {
            if (!used_f[c.fi] && !used_r[c.ri]) {
                used_f[c.fi] = used_r[c.ri] = true;
                out.push_back(compute_pose(front[c.fi], rear[c.ri]));
            }
        }
        return out;
    }

    // Fallback: mutual nearest neighbor
    struct NN { int idx; double d; };
    std::vector<NN> f2r(front.size(), { -1, 1e9 });
    std::vector<NN> r2f(rear.size(), { -1, 1e9 });

    for (int i = 0; i < (int)front.size(); i++) {
        for (int j = 0; j < (int)rear.size(); j++) {
            double d = std::hypot(front[i].x - rear[j].x,
                front[i].y - rear[j].y);
            if (d < f2r[i].d) f2r[i] = { j, d };
            if (d < r2f[j].d) r2f[j] = { i, d };
        }
    }

    std::vector<bool> used_f(front.size(), false), used_r(rear.size(), false);
    for (int i = 0; i < (int)front.size(); i++) {
        int j = f2r[i].idx;
        double d = f2r[i].d;
        if (j < 0 || d > max_pair_px) continue;
        if (r2f[j].idx == i && !used_f[i] && !used_r[j]) {
            used_f[i] = used_r[j] = true;
            out.push_back(compute_pose(front[i], rear[j]));
        }
    }

    return out;
}

std::vector<RobotTrack> Tracker::updateTracks(
    const std::vector<RobotTrack>& prev,
    const std::vector<RobotDet>& dets,
    double now,
    double max_match_dist,
    int max_misses)
{
    std::vector<RobotTrack> tracks = prev;

    struct Pair { double d; int ti; int di; };
    std::vector<Pair> pairs;

    for (int ti = 0; ti < (int)tracks.size(); ti++) {
        for (int di = 0; di < (int)dets.size(); di++) {
            double d = std::hypot(tracks[ti].x - dets[di].x,
                tracks[ti].y - dets[di].y);
            if (d <= max_match_dist) pairs.push_back({ d, ti, di });
        }
    }

    std::sort(pairs.begin(), pairs.end(),
        [](auto& a, auto& b) { return a.d < b.d; });

    std::vector<bool> used_t(tracks.size(), false), used_d(dets.size(), false);

    for (auto& p : pairs) {
        if (!used_t[p.ti] && !used_d[p.di]) {
            used_t[p.ti] = used_d[p.di] = true;
            auto& tr = tracks[p.ti];
            const auto& det = dets[p.di];
            tr.x = det.x;
            tr.y = det.y;
            tr.theta = det.theta;
            tr.sep_px = det.sep_px;
            tr.last_seen = now;
            tr.misses = 0;
        }
    }

    // Increment misses
    for (auto& tr : tracks) {
        if (!used_t[&tr - &tracks[0]]) tr.misses++;
    }

    // Remove lost tracks
    std::vector<RobotTrack> kept;
    for (auto& tr : tracks) {
        if (tr.misses <= max_misses) kept.push_back(tr);
    }
    tracks.swap(kept);

    // Add new tracks
    int next_id = 0;
    for (auto& tr : tracks) next_id = std::max(next_id, tr.id + 1);

    for (int di = 0; di < (int)dets.size(); di++) {
        if (!used_d[di]) {
            const auto& det = dets[di];
            tracks.push_back({ next_id++, det.x, det.y, det.theta,
                              det.sep_px, now, 0 });
        }
    }

    return tracks;
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