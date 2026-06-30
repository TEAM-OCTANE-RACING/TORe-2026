#pragma once

/**
 * cone_depth.hpp
 *
 * This file has NO dependency on calibration.h or rectification.h.
 * It only needs rectified images and camera intrinsics.
 *
 * Workflow:
 *   1. Rectify images via Rectifier (rectification.h)
 *   2. Load intrinsics into CameraConfig from extrinsics.txt
 *   3. Call estimate_depths() with rectified images + YOLO keypoints
 *
 * Example:
 *   // Setup (once per session)
 *   Rectifier rect;
 *   rect.init_from_file("extrinsics.txt");
 *
 *   ConeDepth::CameraConfig cam;
 *   cam.load_from_file("extrinsics.txt");
 *
 *   ConeDepth::PipelineConfig cfg;
 *   cfg.disp_max = (int)(cam.fx * cam.baseline / 0.5) + 30;
 *
 *   // Per frame
 *   cv::Mat rL, rR;
 *   rect.rectify(frameL, frameR, rL, rR);
 *   auto results = ConeDepth::estimate_depths(rL, rR, kps, cls, cam, cfg);
 */

#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include <string>

namespace ConeDepth {

// ─────────────────────────────────────────────────────────────────────────────
// INPUT TYPES  (identical layout to keypoints.h — do not change)
// ─────────────────────────────────────────────────────────────────────────────

struct Point {
    cv::Point2f point;       // pixel in LEFT rectified image
    float       visibility;  // YOLO keypoint confidence [0, 1]
};

struct Keypoints {
    float confidence;        // YOLO detection confidence [0, 1]
    Point p[8];              // p[0],p[1] = ring 0 (top)   left/right edge
                             // p[2],p[3] = ring 1         left/right edge
                             // p[4],p[5] = ring 2         left/right edge
                             // p[6],p[7] = ring 3 (base)  left/right edge
};

// ─────────────────────────────────────────────────────────────────────────────
// CAMERA CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────

/**
 * CameraConfig
 *
 * Holds the four left-camera intrinsics + baseline.
 * Values come from P1 (left rectified projection matrix from stereoRectify):
 *   fx = P1[0][0]   fy = P1[1][1]
 *   cx = P1[0][2]   cy = P1[1][2]
 *   baseline = |T_stereo| in metres (saved as "baseline" key in extrinsics.txt)
 *
 * Two ways to populate:
 *   A) load_from_file("extrinsics.txt")  — reads P1 + baseline from the YAML
 *   B) hardcode(fx, fy, cx, cy, B)       — set directly, no file I/O
 */
struct CameraConfig {
    double fx       = 700.0;
    double fy       = 700.0;
    double cx       = 640.0;
    double cy       = 360.0;
    double baseline = 0.12;   // metres

    /**
     * load_from_file
     * Reads the P1 matrix and baseline key from a YAML file written
     * by save_extrinsics() in calibration.cpp.
     * Returns false if the file cannot be opened or required keys are absent.
     */
    bool load_from_file(const std::string& path = "extrinsics.txt");

    /**
     * hardcode
     * Populate directly with known values — no file I/O at runtime.
     * Use this for deployment once you have confirmed calibration values.
     */
    void hardcode(double fx_, double fy_,
                  double cx_, double cy_,
                  double baseline_) {
        fx = fx_; fy = fy_; cx = cx_; cy = cy_; baseline = baseline_;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// PIPELINE CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────

/**
 * PipelineConfig
 * All tunable parameters. Defaults suit 12 cm baseline, ~700 px focal length,
 * cones at 0.5–20 m.
 *
 * Most important to set correctly:
 *   disp_max = ceil(fx * baseline / min_cone_distance_metres)
 *   e.g. 700 * 0.12 / 0.5 = 168 → set 200 for margin
 */
struct PipelineConfig {
    // Confidence thresholds
    float  det_conf_thresh  = 0.40f;   // min YOLO detection confidence
    float  kp_vis_thresh    = 0.45f;   // min per-keypoint visibility

    // NCC patch: full size = (2*patch_half + 1)^2  →  7 gives 15x15
    int    patch_half       = 7;

    // Disparity search range [disp_min, disp_max] in pixels
    int    disp_min         = 1;
    int    disp_max         = 200;

    // NCC quality gates
    double ncc_thresh       = 0.55;    // min NCC score to accept a match
    double uniqueness_ratio = 1.15;    // best_ncc / second_best_ncc threshold

    // Depth validity gates (metres)
    double min_depth        = 0.3;
    double max_depth        = 25.0;

    // Per-ring aggregation weights (ring 0 = top, ring 3 = base)
    double ring_weight[4]   = { 1.5, 1.3, 1.0, 0.6 };

    // Outlier gate: reject ring if |Z - median_Z| / median_Z > this
    double outlier_z_ratio  = 0.25;
};

// ─────────────────────────────────────────────────────────────────────────────
// INTERMEDIATE TYPES
// ─────────────────────────────────────────────────────────────────────────────

/** Geometry of one horizontal ring band on the cone */
struct Ring {
    cv::Point2f centre;       // midpoint of left + right edge (left image px)
    float       width  = 0;   // right.x - left.x (px), increases top→base
    float       conf   = 0;   // min(visibility_L, visibility_R)
    bool        valid  = false;
};

/** Result of NCC stereo matching for one keypoint */
struct MatchResult {
    double x_right   = -1;   // matched x in right image (subpixel)
    double disparity = -1;   // x_left - x_right  (> 0 for objects in front)
    double ncc_score = -1;   // NCC quality [0, 1]
    bool   valid     = false;
};

/** 3D back-projection of one matched keypoint (camera frame, metres) */
struct KP3D {
    double X         = 0;
    double Y         = 0;
    double Z         = 0;
    double disparity = 0;
    double ncc_score = 0;
    bool   valid     = false;
};

/** All intermediate results for one keypoint */
struct KPResult {
    Point       left_kp;     // original left-image keypoint
    MatchResult match;       // stereo correspondence in right image
    KP3D        pt3d;        // back-projected 3D position
};

// ─────────────────────────────────────────────────────────────────────────────
// PRIMARY OUTPUT
// ─────────────────────────────────────────────────────────────────────────────

/**
 * ConeResult — complete output for one cone detection.
 *
 * Primary outputs:
 *   depth_z   — Z in metres (distance along optical axis)
 *   bearing   — degrees from camera centre-line
 *               negative = cone is left  of centre line
 *               positive = cone is right of centre line
 *   position  — (X, Y, Z) in camera frame metres
 *
 * Diagnostic outputs:
 *   rings[]     — ring geometry from stage 4
 *   kp_res[]    — per-keypoint match + 3D from stages 5–6
 *   valid_rings — number of rings that contributed to depth (0–4)
 *   valid       — false if no confident depth estimate was possible
 */
struct ConeResult {
    int         cone_id    = 0;
    int         cls        = 0;   // cone colour class
    float       det_conf   = 0;

    double      depth_z    = -1;
    double      bearing    = 0;
    cv::Point3d position;

    std::array<Ring,     4> rings;
    std::array<KPResult, 8> kp_res;
    int         valid_rings = 0;
    bool        valid       = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// FUNCTION DECLARATIONS
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Stage 4 — compute_rings
 * Derives 4 ring centres from 8 keypoints.
 * Validates monotonicity: widths must increase ring0 (top) → ring3 (base).
 * Invalid rings are excluded from all subsequent stages.
 */
std::array<Ring, 4> compute_rings(
    const Keypoints&      kps,
    const PipelineConfig& cfg);

/**
 * Stage 5A — ncc_at
 * NCC between patch at (lx,ly) in left_gray and (rx,ry) in right_gray.
 * Returns [-1, 1]. Returns -1.0 if out of bounds; 0.0 if flat patch.
 */
double ncc_at(
    const cv::Mat& left_gray,
    const cv::Mat& right_gray,
    int lx, int ly,
    int rx, int ry,
    int half);

/**
 * Stage 5B — match_keypoint
 * Scans disparity range along epipolar row. Applies NCC threshold and
 * uniqueness check. Refines to subpixel via parabola fit.
 * Returns invalid MatchResult if no confident match found.
 */
MatchResult match_keypoint(
    const cv::Mat&        left_gray,
    const cv::Mat&        right_gray,
    const cv::Point2f&    left_pt,
    const PipelineConfig& cfg);

/**
 * Stage 6A — backproject
 * Z = fx * baseline / disparity
 * X = (u - cx) * Z / fx
 * Y = (v - cy) * Z / fy
 * Returns invalid KP3D if depth outside [min_depth, max_depth].
 */
KP3D backproject(
    const cv::Point2f&    left_pt,
    double                disparity,
    double                ncc_score,
    const CameraConfig&   cam,
    const PipelineConfig& cfg);

/**
 * compute_bearing
 * bearing = atan2(X, Z) in degrees.
 * Negative = left of centre line. Positive = right.
 */
double compute_bearing(double X, double Z);

/**
 * Stage 6B — aggregate_depth
 * Pairs kp_res[2r]/kp_res[2r+1] into ring-centre 3D per ring.
 * Median Z outlier gate. Confidence-weighted mean (X,Y,Z).
 * Writes position, depth_z, bearing, valid_rings into result.
 */
void aggregate_depth(
    ConeResult&           result,
    const CameraConfig&   cam,
    const PipelineConfig& cfg);

/**
 * estimate_depths — top-level per-frame call.
 *
 * @param left_rect   Rectified left  image (BGR or gray, already cv::remap'd)
 * @param right_rect  Rectified right image (BGR or gray, already cv::remap'd)
 * @param keypoints   One Keypoints per detection from detect_keypoints()
 * @param classes     Cone colour class per detection (same order)
 * @param cam         Camera intrinsics + baseline
 * @param cfg         Pipeline parameters
 * @return            One ConeResult per entry in keypoints
 */
std::vector<ConeResult> estimate_depths(
    const cv::Mat&                left_rect,
    const cv::Mat&                right_rect,
    const std::vector<Keypoints>& keypoints,
    const std::vector<int>&       classes,
    const CameraConfig&           cam,
    const PipelineConfig&         cfg);

/**
 * draw_results — draw depth + bearing onto both rectified images in-place.
 * Left:  ring-coloured dots, disparity labels, depth+bearing per cone.
 * Right: matched keypoint positions at same epipolar row.
 */
void draw_results(
    cv::Mat&                       left_vis,
    cv::Mat&                       right_vis,
    const std::vector<ConeResult>& results);

} // namespace ConeDepth