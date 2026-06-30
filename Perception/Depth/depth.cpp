/*
 * depth.cpp
 *
 * Stereo cone depth + bearing pipeline.
 * Consumes Detection (cone.h) and Keypoints (keypoints.h) directly.
 *
 */

#include "cone_depth.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace ConeDepth {

// ─────────────────────────────────────────────────────────────────────────────
// CameraConfig::load_from_file
//
// Reads the P1 matrix and baseline from extrinsics.txt (YAML format)
// written by save_extrinsics() in calibration.cpp.
//
// P1 is the left rectified projection matrix from cv::stereoRectify:
//   [ fx   0  cx   0 ]
//   [  0  fy  cy   0 ]
//   [  0   0   1   0 ]
// ─────────────────────────────────────────────────────────────────────────────

bool CameraConfig::load_from_file(const std::string& path)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ConeDepth] Cannot open: " << path << "\n";
        return false;
    }

    cv::Mat P1_mat;
    fs["P1"]       >> P1_mat;
    fs["baseline"] >> baseline;
    fs.release();

    if (!P1_mat.empty() && P1_mat.rows == 3 && P1_mat.cols == 4) {
        fx = P1_mat.at<double>(0, 0);
        fy = P1_mat.at<double>(1, 1);
        cx = P1_mat.at<double>(0, 2);
        cy = P1_mat.at<double>(1, 2);
    } else {
        // Fallback: individual scalar keys
        cv::FileStorage fs2(path, cv::FileStorage::READ);
        fs2["fx"] >> fx;
        fs2["fy"] >> fy;
        fs2["cx"] >> cx;
        fs2["cy"] >> cy;
        fs2.release();
    }

    if (fx <= 0 || fy <= 0 || baseline <= 0) {
        std::cerr << "[ConeDepth] Bad values in " << path
                  << "  fx=" << fx << " baseline=" << baseline << "\n";
        return false;
    }

    std::cout << "[ConeDepth] Intrinsics loaded:"
              << " fx=" << fx << " fy=" << fy
              << " cx=" << cx << " cy=" << cy
              << " B="  << baseline << "m\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 4 — compute_rings
//
// For ring r (0-3):
//   left  edge = kps.p[2r]    (Point: pixel + visibility)
//   right edge = kps.p[2r+1]
//   centre     = midpoint of left + right pixel coords
//   width      = right.point.x - left.point.x  (must be > 0)
//   conf       = min(visibility_L, visibility_R)
//
// Monotonicity check: width must strictly increase ring0 → ring3.
// Ring with lower confidence in a violating pair is flagged invalid.
// ─────────────────────────────────────────────────────────────────────────────

std::array<Ring, 4> compute_rings(
    const Keypoints&      kps,
    const PipelineConfig& cfg)
{
    std::array<Ring, 4> rings;

    for (int r = 0; r < 4; ++r) {
        const Point& L = kps.p[2 * r];
        const Point& R = kps.p[2 * r + 1];

        // Both keypoints must pass visibility threshold
        if (L.visibility < cfg.kp_vis_thresh) continue;
        if (R.visibility < cfg.kp_vis_thresh) continue;

        // Right edge must be to the RIGHT of left edge
        if (R.point.x <= L.point.x) continue;

        rings[r].centre = cv::Point2f(
            (L.point.x + R.point.x) * 0.5f,
            (L.point.y + R.point.y) * 0.5f);
        rings[r].width  = R.point.x - L.point.x;
        rings[r].conf   = std::min(L.visibility, R.visibility);
        rings[r].valid  = true;
    }

    // Monotonicity: ring[r].width must be greater than ring[r-1].width
    for (int r = 1; r < 4; ++r) {
        if (!rings[r].valid || !rings[r - 1].valid) continue;
        if (rings[r].width < rings[r - 1].width) {
            // Discard whichever ring has lower confidence
            if (rings[r].conf < rings[r - 1].conf)
                rings[r].valid     = false;
            else
                rings[r - 1].valid = false;
        }
    }

    return rings;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 5A — ncc_at
//
// Normalised Cross-Correlation between two (2*half+1)^2 patches.
// NCC = dot(nl, nr) / sqrt(dot(nl,nl) * dot(nr,nr))
//   where nl = left_patch - mean(left_patch)  etc.
//
// Invariant to additive intensity offset between cameras.
// Returns 0.0 for flat patches (no texture → denominator near zero).
// Returns -1.0 if either patch is out of image bounds.
// ─────────────────────────────────────────────────────────────────────────────

double ncc_at(
    const cv::Mat& left_gray,
    const cv::Mat& right_gray,
    int lx, int ly,
    int rx, int ry,
    int half)
{
    const int H  = left_gray.rows;
    const int WL = left_gray.cols;
    const int WR = right_gray.cols;

    if (lx - half < 0 || lx + half >= WL) return -1.0;
    if (ly - half < 0 || ly + half >= H)  return -1.0;
    if (rx - half < 0 || rx + half >= WR) return -1.0;
    if (ry - half < 0 || ry + half >= H)  return -1.0;

    const int sz = 2 * half + 1;

    cv::Mat fl, fr;
    left_gray (cv::Rect(lx - half, ly - half, sz, sz)).convertTo(fl, CV_64F);
    right_gray(cv::Rect(rx - half, ry - half, sz, sz)).convertTo(fr, CV_64F);

    const double ml = cv::mean(fl)[0];
    const double mr = cv::mean(fr)[0];

    const cv::Mat nl = fl - ml;
    const cv::Mat nr = fr - mr;

    const double num   = nl.dot(nr);
    const double denom = std::sqrt(nl.dot(nl) * nr.dot(nr));

    if (denom < 1e-8) return 0.0;   // flat patch — no texture
    return num / denom;             // NCC in [-1, 1]
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 5B — match_keypoint
//
// After rectification: y_right = y_left (same row), x_right = x_left - d.
//
// Steps:
//   1. Integer scan over disparity range [disp_min, disp_max]
//   2. NCC threshold gate: best_ncc must exceed ncc_thresh
//   3. Uniqueness gate: best_ncc / second_best_ncc > uniqueness_ratio
//      (rejects ambiguous matches on repetitive cone stripe texture)
//   4. Parabola subpixel fit on scores at (best_d-1, best_d, best_d+1)
//      Improves disparity precision from ±0.5 px to ~0.1 px
// ─────────────────────────────────────────────────────────────────────────────

MatchResult match_keypoint(
    const cv::Mat&        left_gray,
    const cv::Mat&        right_gray,
    const cv::Point2f&    left_pt,
    const PipelineConfig& cfg)
{
    MatchResult result;

    const int lx   = static_cast<int>(std::round(left_pt.x));
    const int ly   = static_cast<int>(std::round(left_pt.y));
    const int half = cfg.patch_half;

    double best_ncc   = -2.0;
    double second_ncc = -2.0;
    int    best_d     = -1;

    for (int d = cfg.disp_min; d <= cfg.disp_max; ++d) {
        const double s = ncc_at(left_gray, right_gray,
                                 lx, ly,
                                 lx - d, ly,   // epipolar: same row, x shifted
                                 half);
        if (s > best_ncc) {
            second_ncc = best_ncc;
            best_ncc   = s;
            best_d     = d;
        } else if (s > second_ncc) {
            second_ncc = s;
        }
    }

    // Gate 1: minimum NCC quality
    if (best_d < 0 || best_ncc < cfg.ncc_thresh)
        return result;

    // Gate 2: uniqueness — reject ambiguous matches
    if (second_ncc > -1.5 &&
        best_ncc / (second_ncc + 1e-9) < cfg.uniqueness_ratio)
        return result;

    // Subpixel parabola fit: peak at d_sub = best_d - 0.5*(s_p - s_m)/(s_p - 2*s_c + s_m)
    double sub_d = static_cast<double>(best_d);
    if (best_d > cfg.disp_min && best_d < cfg.disp_max) {
        const double s_m = ncc_at(left_gray, right_gray,
                                   lx, ly, lx - (best_d - 1), ly, half);
        const double s_p = ncc_at(left_gray, right_gray,
                                   lx, ly, lx - (best_d + 1), ly, half);
        const double denom_par = s_p - 2.0 * best_ncc + s_m;
        if (std::abs(denom_par) > 1e-8)
            sub_d = best_d - 0.5 * (s_p - s_m) / denom_par;
    }

    const double x_right = static_cast<double>(lx) - sub_d;

    // Sanity: x_right must be left of x_left and inside image
    if (x_right >= static_cast<double>(lx)) return result;
    if (x_right < 0.0)                      return result;

    result.x_right   = x_right;
    result.disparity = sub_d;
    result.ncc_score = best_ncc;
    result.valid     = true;
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 6A — backproject
//
//   Z = fx * baseline / disparity   (stereo depth equation)
//   X = (u - cx) * Z / fx           (horizontal position, right = positive)
//   Y = (v - cy) * Z / fy           (vertical position,   down  = positive)
// ─────────────────────────────────────────────────────────────────────────────

KP3D backproject(
    const cv::Point2f&    left_pt,
    double                disparity,
    double                ncc_score,
    const CameraConfig&   cam,
    const PipelineConfig& cfg)
{
    KP3D p;
    if (disparity <= 0.0) return p;

    const double Z = (cam.fx * cam.baseline) / disparity;
    if (Z < cfg.min_depth || Z > cfg.max_depth) return p;

    p.X         = (left_pt.x - cam.cx) * Z / cam.fx;
    p.Y         = (left_pt.y - cam.cy) * Z / cam.fy;
    p.Z         = Z;
    p.disparity = disparity;
    p.ncc_score = ncc_score;
    p.valid     = true;
    return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_bearing
// ─────────────────────────────────────────────────────────────────────────────

double compute_bearing(double X, double Z)
{
    // atan2(X, Z): angle from forward axis (Z) to lateral offset (X)
    // Negative X → left  → negative bearing
    // Positive X → right → positive bearing
    return std::atan2(X, Z) * (180.0 / M_PI);
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 6B — aggregate_depth
//
// Builds a ring-centre 3D from each kp pair, applies median-Z outlier gate,
// then computes a weighted mean with:
//   weight = ring_weight[r] * ncc_score * ring_conf
// ─────────────────────────────────────────────────────────────────────────────

void aggregate_depth(
    ConeResult&           result,
    const CameraConfig&   /*cam*/,
    const PipelineConfig& cfg)
{
    struct RingCentre {
        double X = 0, Y = 0, Z = 0;
        double ncc  = 0;
        double conf = 0;
        bool   valid = false;
    };
    std::array<RingCentre, 4> rc;

    for (int r = 0; r < 4; ++r) {
        const KP3D& pl = result.kp_res[2 * r].pt3d;
        const KP3D& pr = result.kp_res[2 * r + 1].pt3d;

        const double ring_conf =
            result.rings[r].valid
            ? static_cast<double>(result.rings[r].conf)
            : 0.5;

        if (pl.valid && pr.valid) {
            rc[r] = {
                (pl.X + pr.X) * 0.5,
                (pl.Y + pr.Y) * 0.5,
                (pl.Z + pr.Z) * 0.5,
                (pl.ncc_score + pr.ncc_score) * 0.5,
                ring_conf, true
            };
        } else if (pl.valid) {
            rc[r] = { pl.X, pl.Y, pl.Z, pl.ncc_score, ring_conf, true };
        } else if (pr.valid) {
            rc[r] = { pr.X, pr.Y, pr.Z, pr.ncc_score, ring_conf, true };
        }
    }

    // Collect valid Z values and compute median for outlier gating
    std::vector<double> z_vals;
    z_vals.reserve(4);
    for (int r = 0; r < 4; ++r)
        if (rc[r].valid) z_vals.push_back(rc[r].Z);

    if (z_vals.empty()) return;

    std::vector<double> sorted_z = z_vals;
    std::nth_element(sorted_z.begin(),
                     sorted_z.begin() + sorted_z.size() / 2,
                     sorted_z.end());
    const double med_z = sorted_z[sorted_z.size() / 2];

    // Outlier-gated weighted mean
    double sum_w = 0, sum_X = 0, sum_Y = 0, sum_Z = 0;
    int    valid_count = 0;

    for (int r = 0; r < 4; ++r) {
        if (!rc[r].valid) continue;

        if (std::abs(rc[r].Z - med_z) / (med_z + 1e-8) > cfg.outlier_z_ratio)
            continue;

        const double w = cfg.ring_weight[r] * rc[r].ncc * rc[r].conf;
        if (w <= 0.0) continue;

        sum_X += rc[r].X * w;
        sum_Y += rc[r].Y * w;
        sum_Z += rc[r].Z * w;
        sum_w += w;
        ++valid_count;
    }

    if (sum_w < 1e-9 || valid_count == 0) return;

    const double iw    = 1.0 / sum_w;
    result.position    = cv::Point3d(sum_X * iw, sum_Y * iw, sum_Z * iw);
    result.depth_z     = result.position.z;
    result.bearing     = compute_bearing(result.position.x, result.position.z);
    result.valid_rings = valid_count;
    result.valid       = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// estimate_depths — top-level per-frame call
// ─────────────────────────────────────────────────────────────────────────────

std::vector<ConeResult> estimate_depths(
    const cv::Mat&                 left_rect,
    const cv::Mat&                 right_rect,
    const std::vector<Detection>&  detections,
    const std::vector<Keypoints>&  keypoints,
    const CameraConfig&            cam,
    const PipelineConfig&          cfg)
{
    // Both vectors must be parallel — same size, same order
    const int N = static_cast<int>(detections.size());

    // Pre-allocate output parallel to detections (invalid by default)
    std::vector<ConeResult> results(N);
    for (int i = 0; i < N; ++i) {
        results[i].class_id  = detections[i].class_id;
        results[i].label     = detections[i].label;
        results[i].det_conf  = detections[i].confidence;
        results[i].box       = detections[i].box;
    }

    // Validate sizes match
    if ((int)keypoints.size() != N) {
        std::cerr << "[ConeDepth] detections.size()=" << N
                  << " but keypoints.size()=" << keypoints.size()
                  << " — vectors must be parallel.\n";
        return results;
    }

    // Convert to grayscale — NCC operates on intensity
    cv::Mat Lg, Rg;
    if (left_rect.channels() == 3) {
        cv::cvtColor(left_rect,  Lg, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_rect, Rg, cv::COLOR_BGR2GRAY);
    } else {
        Lg = left_rect;
        Rg = right_rect;
    }

    for (int i = 0; i < N; ++i) {
        const Detection& det = detections[i];
        const Keypoints& kps = keypoints[i];

        // Skip if detection confidence too low
        if (det.confidence < cfg.det_conf_thresh) continue;

        // Skip if keypoint detection found nothing in this ROI
        if (kps.confidence < 0.0f) continue;

        ConeResult& res = results[i];

        // ── Stage 4: ring centres ─────────────────────────────────────────
        res.rings = compute_rings(kps, cfg);

        // ── Stages 5 + 6A: match + back-project each keypoint ────────────
        for (int k = 0; k < 8; ++k) {
            KPResult& kr = res.kp_res[k];
            kr.left_kp   = kps.p[k];   // Point: .point (pixel) + .visibility

            if (kps.p[k].visibility < cfg.kp_vis_thresh) continue;

            // Stage 5: NCC scan along epipolar row → subpixel disparity
            kr.match = match_keypoint(Lg, Rg, kps.p[k].point, cfg);
            if (!kr.match.valid) continue;

            // Stage 6A: disparity → (X, Y, Z) in camera frame
            kr.pt3d = backproject(
                kps.p[k].point,
                kr.match.disparity,
                kr.match.ncc_score,
                cam, cfg);
        }

        // ── Stage 6B: aggregate ring 3D positions → cone depth + bearing ─
        aggregate_depth(res, cam, cfg);
    }

    return results;
}

// ─────────────────────────────────────────────────────────────────────────────
// draw_results — visualisation
// ─────────────────────────────────────────────────────────────────────────────

void draw_results(
    cv::Mat&                        left_vis,
    cv::Mat&                        right_vis,
    const std::vector<ConeResult>&  results)
{
    // Matches ring colours from keypoints.cpp for visual consistency
    static const cv::Scalar ring_col[4] = {
        { 0,   255, 255 },   // ring 0 — yellow  (matches KEYPOINT_COLORS)
        { 0,   165, 255 },   // ring 1 — orange
        { 0,     0, 255 },   // ring 2 — red
        { 255,   0,   0 },   // ring 3 — blue
    };

    for (const ConeResult& c : results) {
        float ann_x = -1, ann_y = 1e6f;

        for (int k = 0; k < 8; ++k) {
            const KPResult&   kr  = c.kp_res[k];
            const cv::Scalar& col = ring_col[k / 2];

            if (kr.left_kp.visibility < 0.1f) continue;

            const cv::Point2f lpt = kr.left_kp.point;
            cv::circle(left_vis, lpt, 4, col, -1);

            // Track topmost keypoint for annotation placement
            if (lpt.y < ann_y) { ann_y = lpt.y; ann_x = lpt.x; }

            if (kr.match.valid) {
                // Right image: matched point at same row, shifted x
                const cv::Point2f rpt(
                    static_cast<float>(kr.match.x_right), lpt.y);
                cv::circle(right_vis, rpt, 4, col, -1);

                // Disparity + NCC label on left image
                char buf[32];
                std::snprintf(buf, sizeof(buf),
                              "d%.1f", kr.match.disparity);
                cv::putText(left_vis, buf,
                    cv::Point(static_cast<int>(lpt.x) + 6,
                              static_cast<int>(lpt.y) - 3),
                    cv::FONT_HERSHEY_SIMPLEX, 0.28, col, 1);

                // Per-kp Z on right image
                if (kr.pt3d.valid) {
                    char zb[16];
                    std::snprintf(zb, sizeof(zb), "%.2f", kr.pt3d.Z);
                    cv::putText(right_vis, zb,
                        cv::Point(static_cast<int>(rpt.x) + 4,
                                  static_cast<int>(rpt.y) - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.28, col, 1);
                }
            } else {
                // Mark failed match
                cv::drawMarker(left_vis, lpt,
                    cv::Scalar(100, 100, 100),
                    cv::MARKER_TILTED_CROSS, 7, 1);
            }
        }

        // Cone-level depth + bearing label above topmost keypoint
        if (c.valid && ann_x >= 0) {
            const char* side = c.bearing >= 0.0 ? "R" : "L";
            char buf[80];
            std::snprintf(buf, sizeof(buf),
                          "%s  Z=%.2fm  %s%.1fdeg  [%d/4]",
                          c.label.c_str(),
                          c.depth_z,
                          side, std::abs(c.bearing),
                          c.valid_rings);
            cv::putText(left_vis, buf,
                cv::Point(static_cast<int>(ann_x),
                          static_cast<int>(ann_y) - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.38,
                cv::Scalar(255, 255, 255), 1);
        }
    }
}

} // namespace ConeDepth