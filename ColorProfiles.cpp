#include "ColorProfiles.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <tuple>

namespace ColorProfiles {
namespace {

void rgb_to_hsv(ibyte R, ibyte G, ibyte B, double& h, double& s, double& v)
{
    int maxv = R, minv = R;
    if (G > maxv) maxv = G;
    if (B > maxv) maxv = B;
    if (G < minv) minv = G;
    if (B < minv) minv = B;

    int delta = maxv - minv;
    v = maxv;

    if (delta == 0) {
        s = 0.0;
        h = 0.0;
        return;
    }

    s = (maxv == 0) ? 0.0 : (delta / static_cast<double>(maxv));

    double H;
    if (maxv == R)      H = static_cast<double>(G - B) / delta;
    else if (maxv == G) H = static_cast<double>(B - R) / delta + 2.0;
    else                H = static_cast<double>(R - G) / delta + 4.0;

    h = 60.0 * H;
    if (h < 0.0) h += 360.0;
}

bool hue_in_ranges(double h, const std::vector<HSVRange>& ranges)
{
    for (const auto& r : ranges) {
        if (r.h_lo <= r.h_hi) {
            if (h >= r.h_lo && h <= r.h_hi) return true;
        } else {
            if (h >= r.h_lo || h <= r.h_hi) return true;
        }
    }
    return false;
}

void build_mask(image& rgb,
                image& mask_out,
                const std::vector<HSVRange>& hue_ranges,
                double s_min,
                int v_min)
{
    const int W = rgb.width;
    const int H = rgb.height;
    std::memset(mask_out.pdata, 0, static_cast<size_t>(W) * H);

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            const int idx = 3 * (j * W + i);
            ibyte B = rgb.pdata[idx + 0];
            ibyte G = rgb.pdata[idx + 1];
            ibyte R = rgb.pdata[idx + 2];

            double h, s, v;
            rgb_to_hsv(R, G, B, h, s, v);

            mask_out.pdata[j * W + i] =
                (s >= s_min && v >= v_min && hue_in_ranges(h, hue_ranges)) ? 255 : 0;
        }
    }
}

void clean_mask(image& mask, int open_iters, int close_iters, int repeat)
{
    if (repeat <= 0) repeat = 1;

    image tmp;
    tmp.type = GREY_IMAGE;
    tmp.width = mask.width;
    tmp.height = mask.height;
    allocate_image(tmp);

    for (int rep = 0; rep < repeat; ++rep) {
        for (int it = 0; it < open_iters; ++it) {
            erode(mask, tmp);
            copy(tmp, mask);
        }
        for (int it = 0; it < open_iters; ++it) {
            dialate(mask, tmp);
            copy(tmp, mask);
        }
        for (int it = 0; it < close_iters; ++it) {
            dialate(mask, tmp);
            copy(tmp, mask);
        }
        for (int it = 0; it < close_iters; ++it) {
            erode(mask, tmp);
            copy(tmp, mask);
        }
    }

    free_image(tmp);
}

std::vector<Blob> extract_blobs_filtered(image& mask,
                                         image& grey_for_centroid,
                                         int min_blob_area,
                                         int max_blob_area,
                                         double min_area_ratio)
{
    std::vector<Blob> out_blobs;

    image label;
    label.type = LABEL_IMAGE;
    label.width = mask.width;
    label.height = mask.height;
    allocate_image(label);

    int nlabels = 0;
    label_image(mask, label, nlabels);

    i2byte* pl = reinterpret_cast<i2byte*>(label.pdata);
    const int W = label.width;
    const int H = label.height;

    for (int L = 1; L <= nlabels; ++L) {
        int area = 0;
        int minx = W, miny = H, maxx = 0, maxy = 0;

        for (int j = 0; j < H; ++j) {
            for (int i = 0; i < W; ++i) {
                if (pl[j * W + i] == L) {
                    ++area;
                    if (i < minx) minx = i;
                    if (i > maxx) maxx = i;
                    if (j < miny) miny = j;
                    if (j > maxy) maxy = j;
                }
            }
        }

        if (area < min_blob_area) continue;
        if (max_blob_area > 0 && area > max_blob_area) continue;

        const int bw = std::max(1, maxx - minx + 1);
        const int bh = std::max(1, maxy - miny + 1);
        const double area_ratio = static_cast<double>(area) / static_cast<double>(bw * bh);
        if (area_ratio < min_area_ratio) continue;

        double ic, jc;
        centroid(grey_for_centroid, label, L, ic, jc);

        Blob b{};
        b.x = ic;
        b.y = jc;
        b.area = area;
        out_blobs.push_back(b);
    }

    free_image(label);
    std::sort(out_blobs.begin(), out_blobs.end(), [](const Blob& a, const Blob& b) {
        return a.area > b.area;
    });
    return out_blobs;
}

ProfiledRobotDet compute_pose(const Blob& front,
                              const Blob& rear,
                              const RobotColorProfile& profile,
                              double pair_score)
{
    const double dx = front.x - rear.x;
    const double dy = front.y - rear.y;

    ProfiledRobotDet out{};
    out.profile_name = profile.name;
    out.front_color = profile.front_color;
    out.rear_color = profile.rear_color;
    out.front = front;
    out.rear = rear;
    out.x = 0.5 * (front.x + rear.x);
    out.y = 0.5 * (front.y + rear.y);
    out.theta = std::atan2(dy, dx);
    out.sep_px = std::hypot(dx, dy);
    out.pair_score = pair_score;
    return out;
}

} // namespace

std::map<std::string, ColorSpec> buildDefaultColorSpecs()
{
    std::map<std::string, ColorSpec> specs;
    specs["green"] = ColorSpec{"green", {HSVRange{35.0, 95.0, 0.0, 0}}, 0.45, 35};
    specs["red"]   = ColorSpec{"red", {HSVRange{0.0, 10.0, 0.0, 0}, HSVRange{172.0, 179.0, 0.0, 0}}, 0.90, 40};
    specs["orange"] = ColorSpec{"orange", {HSVRange{8.0, 28.0, 0.0, 0}}, 0.70, 50};
    specs["blue"]   = ColorSpec{"blue", {HSVRange{95.0, 140.0, 0.0, 0}}, 0.30, 20};
    return specs;
}

std::map<std::string, RobotColorProfile> buildDefaultRobotProfiles()
{
    std::map<std::string, RobotColorProfile> profiles;
    profiles["GR"] = RobotColorProfile{"GR", "green", "red"};
    profiles["OB"] = RobotColorProfile{"OB", "orange", "blue"};
    return profiles;
}

void setColorSpec(std::map<std::string, ColorSpec>& specs,
                  const std::string& name,
                  const std::vector<HSVRange>& hue_ranges,
                  double s_min,
                  int v_min)
{
    specs[name] = ColorSpec{name, hue_ranges, s_min, v_min};
}

std::map<std::string, image> segmentAllMarkerColors(
    image& rgb,
    const std::map<std::string, ColorSpec>& color_specs,
    int morph_open_iters,
    int morph_close_iters,
    int morph_repeat)
{
    std::map<std::string, image> masks;
    for (const auto& kv : color_specs) {
        image mask{};
        mask.type = GREY_IMAGE;
        mask.width = rgb.width;
        mask.height = rgb.height;
        allocate_image(mask);

        build_mask(rgb, mask, kv.second.hue_ranges, kv.second.s_min, kv.second.v_min);
        clean_mask(mask, morph_open_iters, morph_close_iters, morph_repeat);
        masks[kv.first] = mask;
    }
    return masks;
}

void freeMaskMap(std::map<std::string, image>& masks)
{
    for (auto& kv : masks) {
        free_image(kv.second);
    }
    masks.clear();
}

std::map<std::string, std::vector<Blob>> extractBlobsByColor(
    image& rgb,
    const std::map<std::string, image>& masks,
    int min_blob_area,
    int max_blob_area,
    double min_area_ratio)
{
    std::map<std::string, std::vector<Blob>> out;

    image grey{};
    grey.type = GREY_IMAGE;
    grey.width = rgb.width;
    grey.height = rgb.height;
    allocate_image(grey);
    copy(rgb, grey);

    for (const auto& kv : masks) {
        image mask_copy{};
        mask_copy.type = GREY_IMAGE;
        mask_copy.width = kv.second.width;
        mask_copy.height = kv.second.height;
        allocate_image(mask_copy);
        copy(const_cast<image&>(kv.second), mask_copy);

        out[kv.first] = extract_blobs_filtered(mask_copy, grey, min_blob_area, max_blob_area, min_area_ratio);
        free_image(mask_copy);
    }

    free_image(grey);
    return out;
}

std::optional<double> estimateMarkerSepPxForProfile(
    const std::vector<Blob>& front_blobs,
    const std::vector<Blob>& rear_blobs)
{
    if (front_blobs.empty() || rear_blobs.empty()) return std::nullopt;

    std::vector<double> dists;
    dists.reserve(front_blobs.size());
    for (const auto& f : front_blobs) {
        double best = std::numeric_limits<double>::infinity();
        for (const auto& r : rear_blobs) {
            const double d = std::hypot(f.x - r.x, f.y - r.y);
            if (d < best) best = d;
        }
        if (std::isfinite(best)) dists.push_back(best);
    }

    if (dists.empty()) return std::nullopt;
    std::sort(dists.begin(), dists.end());
    const size_t n = dists.size();
    if (n % 2 == 1) return dists[n / 2];
    return 0.5 * (dists[n / 2 - 1] + dists[n / 2]);
}

std::map<std::string, std::optional<double>> estimateProfileSeparations(
    const std::map<std::string, RobotColorProfile>& robot_profiles,
    const std::map<std::string, std::vector<Blob>>& blobs_by_color)
{
    std::map<std::string, std::optional<double>> out;
    for (const auto& kv : robot_profiles) {
        const auto& profile = kv.second;
        auto fit = blobs_by_color.find(profile.front_color);
        auto rit = blobs_by_color.find(profile.rear_color);
        if (fit == blobs_by_color.end() || rit == blobs_by_color.end()) {
            out[kv.first] = std::nullopt;
            continue;
        }
        out[kv.first] = estimateMarkerSepPxForProfile(fit->second, rit->second);
    }
    return out;
}

std::vector<ProfiledRobotDet> pairProfileMarkers(
    const std::vector<Blob>& front_blobs,
    const std::vector<Blob>& rear_blobs,
    const RobotColorProfile& profile,
    std::optional<double> expected_sep_px,
    double sep_tol,
    double max_pair_px,
    double area_ratio_tol)
{
    std::vector<ProfiledRobotDet> dets;
    if (front_blobs.empty() || rear_blobs.empty()) return dets;

    struct Candidate {
        double score;
        size_t fi;
        size_t ri;
    };

    std::vector<Candidate> candidates;
    for (size_t i = 0; i < front_blobs.size(); ++i) {
        for (size_t j = 0; j < rear_blobs.size(); ++j) {
            const double d = std::hypot(front_blobs[i].x - rear_blobs[j].x,
                                        front_blobs[i].y - rear_blobs[j].y);
            if (d > max_pair_px) continue;

            double sep_err = 0.0;
            if (expected_sep_px.has_value()) {
                const double expected = expected_sep_px.value();
                const double dmin = (1.0 - sep_tol) * expected;
                const double dmax = (1.0 + sep_tol) * expected;
                if (d < dmin || d > dmax) continue;
                sep_err = std::abs(d - expected) / std::max(1.0, expected);
            }

            const double amin = std::max(1.0, std::min(front_blobs[i].area, rear_blobs[j].area));
            const double amax = std::max(front_blobs[i].area, rear_blobs[j].area);
            const double area_ratio = amax / amin;
            if (area_ratio > area_ratio_tol) continue;

            const double score = sep_err + 0.15 * std::abs(std::log(std::max(1.0, area_ratio)));
            candidates.push_back(Candidate{score, i, j});
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.score < b.score;
    });

    std::vector<char> used_f(front_blobs.size(), 0);
    std::vector<char> used_r(rear_blobs.size(), 0);
    for (const auto& c : candidates) {
        if (used_f[c.fi] || used_r[c.ri]) continue;
        used_f[c.fi] = 1;
        used_r[c.ri] = 1;
        dets.push_back(compute_pose(front_blobs[c.fi], rear_blobs[c.ri], profile, c.score));
    }
    return dets;
}

std::vector<ProfiledRobotDet> detectProfileRobots(
    image& rgb,
    const std::map<std::string, ColorSpec>& color_specs,
    const std::map<std::string, RobotColorProfile>& robot_profiles,
    const std::map<std::string, std::optional<double>>& expected_sep_px_map,
    int blob_min_area,
    int blob_max_area,
    double min_area_ratio,
    double sep_tol,
    double max_pair_px,
    double area_ratio_tol,
    int morph_open_iters,
    int morph_close_iters,
    int morph_repeat,
    std::map<std::string, image>* out_masks,
    std::map<std::string, std::vector<Blob>>* out_blobs_by_color,
    std::map<std::string, std::optional<double>>* out_sep_estimates)
{
    std::map<std::string, image> local_masks =
        segmentAllMarkerColors(rgb, color_specs, morph_open_iters, morph_close_iters, morph_repeat);

    std::map<std::string, std::vector<Blob>> local_blobs =
        extractBlobsByColor(rgb, local_masks, blob_min_area, blob_max_area, min_area_ratio);

    std::map<std::string, std::optional<double>> local_sep_estimates =
        estimateProfileSeparations(robot_profiles, local_blobs);

    std::vector<ProfiledRobotDet> all_dets;
    for (const auto& kv : robot_profiles) {
        const auto& profile = kv.second;
        const auto fit = local_blobs.find(profile.front_color);
        const auto rit = local_blobs.find(profile.rear_color);
        const std::vector<Blob> empty;
        const std::vector<Blob>& front = (fit == local_blobs.end()) ? empty : fit->second;
        const std::vector<Blob>& rear  = (rit == local_blobs.end()) ? empty : rit->second;

        auto it_sep = expected_sep_px_map.find(profile.name);
        std::optional<double> expected = (it_sep == expected_sep_px_map.end()) ? std::nullopt : it_sep->second;

        auto dets = pairProfileMarkers(front, rear, profile, expected, sep_tol, max_pair_px, area_ratio_tol);
        all_dets.insert(all_dets.end(), dets.begin(), dets.end());
    }

    // Reject overlapping profile hypotheses and keep the best-scored one.
    std::sort(all_dets.begin(), all_dets.end(), [](const ProfiledRobotDet& a, const ProfiledRobotDet& b) {
        return a.pair_score < b.pair_score;
    });

    std::vector<ProfiledRobotDet> keep;
    for (const auto& det : all_dets) {
        bool conflict = false;
        for (const auto& kept : keep) {
            const double d = std::hypot(det.x - kept.x, det.y - kept.y);
            if (d < 0.45 * std::max(det.sep_px, kept.sep_px)) {
                conflict = true;
                break;
            }
        }
        if (!conflict) keep.push_back(det);
    }

    if (out_masks) {
        *out_masks = local_masks;
    } else {
        freeMaskMap(local_masks);
    }

    if (out_blobs_by_color) {
        *out_blobs_by_color = local_blobs;
    }
    if (out_sep_estimates) {
        *out_sep_estimates = local_sep_estimates;
    }

    return keep;
}

std::vector<ProfiledRobotTrack> updateProfileTracks(
    const std::vector<ProfiledRobotTrack>& prev,
    const std::vector<ProfiledRobotDet>& dets,
    double now,
    double max_match_dist_px,
    int max_misses)
{
    std::vector<ProfiledRobotTrack> tracks = prev;

    struct Pair {
        double d;
        int ti;
        int di;
    };

    std::vector<Pair> pairs;
    for (int ti = 0; ti < static_cast<int>(tracks.size()); ++ti) {
        for (int di = 0; di < static_cast<int>(dets.size()); ++di) {
            if (tracks[ti].profile_name != dets[di].profile_name) continue;
            const double d = std::hypot(tracks[ti].x - dets[di].x,
                                        tracks[ti].y - dets[di].y);
            if (d <= max_match_dist_px) pairs.push_back(Pair{d, ti, di});
        }
    }

    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
        return a.d < b.d;
    });

    std::vector<char> used_t(tracks.size(), 0);
    std::vector<char> used_d(dets.size(), 0);

    for (const auto& p : pairs) {
        if (used_t[p.ti] || used_d[p.di]) continue;
        used_t[p.ti] = 1;
        used_d[p.di] = 1;

        auto& tr = tracks[p.ti];
        const auto& det = dets[p.di];
        tr.profile_name = det.profile_name;
        tr.front_color = det.front_color;
        tr.rear_color = det.rear_color;
        tr.x = det.x;
        tr.y = det.y;
        tr.theta = det.theta;
        tr.sep_px = det.sep_px;
        tr.last_seen = now;
        tr.misses = 0;
        tr.stable_hits += 1;
    }

    for (size_t i = 0; i < tracks.size(); ++i) {
        if (!used_t[i]) tracks[i].misses += 1;
    }

    std::vector<ProfiledRobotTrack> kept;
    for (const auto& tr : tracks) {
        if (tr.misses <= max_misses) kept.push_back(tr);
    }
    tracks.swap(kept);

    int next_id = 0;
    for (const auto& tr : tracks) next_id = std::max(next_id, tr.id + 1);

    for (int di = 0; di < static_cast<int>(dets.size()); ++di) {
        if (used_d[di]) continue;
        const auto& det = dets[di];
        tracks.push_back(ProfiledRobotTrack{
            next_id++,
            det.profile_name,
            det.front_color,
            det.rear_color,
            det.x,
            det.y,
            det.theta,
            det.sep_px,
            now,
            0,
            1
        });
    }

    return tracks;
}

} // namespace ColorProfiles

