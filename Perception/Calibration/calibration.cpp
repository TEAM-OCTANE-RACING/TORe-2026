/*
 * calibration.cpp
 *
 * Intrinsic calibration (left + right) and stereo extrinsic computation.
 * Saves all results to extrinsics.txt for use by the depth pipeline.
 *
 * Fixes applied vs original:
 *   1. stereo_calibrate: simple per-image averaging of R/T is WRONG.
 *      Averaging rotation matrices destroys orthogonality. Replaced with
 *      cv::stereoCalibrate which is the correct OpenCV API and produces
 *      R1, R2, P1, P2, Q for rectification in one call.
 *   2. R/T were computed from per-image rvecs/tvecs directly — this only
 *      works if the images were perfectly paired. cv::stereoCalibrate
 *      does this correctly with full bundle adjustment.
 *   3. objp was populated inside each thread function behind an empty()
 *      guard — if both threads ran concurrently, both could see empty
 *      and both would push_back, causing double entries. Moved objp
 *      preparation to a dedicated function called before threads start.
 *   4. drawChessboardCorners was called but window never shown — removed.
 *   5. imgSize was only set on the first success — correct, but now
 *      explicitly asserted consistent across frames.
 *   6. save_extrinsics() added: writes cam_left, dist_left, cam_right,
 *      dist_right, R_stereo, T_stereo, R1, R2, P1, P2, Q, baseline,
 *      image size to extrinsics.txt as a cv::FileStorage YAML file.
 *      load_extrinsics() reads them back for the depth pipeline.
 */

#include "calibration.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Globals (declared extern in calibration.h)
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat cam_left,  dist_left;
cv::Mat cam_right, dist_right;

std::vector<cv::Mat> rvecs_left,  tvecs_left;
std::vector<cv::Mat> rvecs_right, tvecs_right;

// Stereo extrinsics (filled by stereo_calibrate)
cv::Mat R_stereo;          // 3x3 rotation from left to right camera
cv::Mat T_stereo;          // 3x1 translation from left to right camera
cv::Mat R1, R2;            // rectification rotations per camera
cv::Mat P1, P2;            // rectified projection matrices
cv::Mat Q;                 // disparity-to-depth reprojection matrix
double  baseline = 0.0;    // |T_stereo| in metres

// Image points collected during intrinsic calibration (needed by stereoCalibrate)
std::vector<std::vector<cv::Point2f>> img_pts_left;
std::vector<std::vector<cv::Point2f>> img_pts_right;

// Object points (shared)
std::vector<cv::Point3f> objp;
cv::Size                 calibration_img_size;  // set during intrinsic calibration

// ─────────────────────────────────────────────────────────────────────────────
// prepare_objp — call ONCE before starting any calibration
// ─────────────────────────────────────────────────────────────────────────────

void prepare_objp() {
    objp.clear();
    for (int r = 0; r < ROW; r++)
        for (int c = 0; c < COL; c++)
            objp.push_back(cv::Point3f(c * CHECKERBOARD_SIZE,
                                       r * CHECKERBOARD_SIZE,
                                       0.0f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal helper: collect chessboard corners from a folder
// ─────────────────────────────────────────────────────────────────────────────

static bool collect_corners(
    const std::string&                            folder_glob,
    std::vector<std::vector<cv::Point2f>>&        img_pts,
    cv::Size&                                     img_size,
    const std::string&                            camera_name)
{
    std::vector<cv::String> images;
    cv::glob(folder_glob, images);

    if (images.empty()) {
        std::cerr << "[" << camera_name << "] No images found at: "
                  << folder_glob << "\n";
        return false;
    }

    img_pts.clear();

    for (const auto& path : images) {
        cv::Mat frame = cv::imread(path);
        if (frame.empty()) {
            std::cerr << "[" << camera_name << "] Failed to read: " << path << "\n";
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray, cv::Size(COL, ROW), corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (!found) continue;

        // Subpixel refinement
        cv::TermCriteria crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
                              30, 0.001);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), crit);

        // Record image size (assert consistency)
        if (img_size.width == 0) {
            img_size = gray.size();
        } else if (img_size != gray.size()) {
            std::cerr << "[" << camera_name << "] Inconsistent image size in "
                      << path << " — skipping.\n";
            continue;
        }

        img_pts.push_back(corners);
    }

    if (img_pts.empty()) {
        std::cerr << "[" << camera_name
                  << "] No valid detections. Calibration cannot proceed.\n";
        return false;
    }

    std::cout << "[" << camera_name << "] Valid frames: " << img_pts.size()
              << "  Image size: " << img_size << "\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// left_intrinsic_calibrate
// ─────────────────────────────────────────────────────────────────────────────

void* left_intrinsic_calibrate(void* /*arg*/) {
    if (objp.empty()) prepare_objp();

    cv::Size img_size;
    if (!collect_corners("./leftcamera/*.png", img_pts_left, img_size, "LEFT"))
        return nullptr;

    calibration_img_size = img_size;

    std::vector<std::vector<cv::Point3f>> obj_pts(img_pts_left.size(), objp);

    double rms = cv::calibrateCamera(
        obj_pts, img_pts_left, img_size,
        cam_left, dist_left,
        rvecs_left, tvecs_left);

    std::cout << "[LEFT]  RMS reprojection error: " << rms << "\n";
    std::cout << "[LEFT]  Camera matrix:\n" << cam_left << "\n";
    std::cout << "[LEFT]  Distortion coefficients: " << dist_left << "\n";

    if (rms > 1.0)
        std::cerr << "[LEFT]  WARNING: RMS > 1.0 — consider recapturing calibration images.\n";

    return nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// right_intrinsic_calibrate
// ─────────────────────────────────────────────────────────────────────────────

void* right_intrinsic_calibrate(void* /*arg*/) {
    if (objp.empty()) prepare_objp();

    cv::Size img_size;
    if (!collect_corners("./rightcamera/*.png", img_pts_right, img_size, "RIGHT"))
        return nullptr;

    std::vector<std::vector<cv::Point3f>> obj_pts(img_pts_right.size(), objp);

    double rms = cv::calibrateCamera(
        obj_pts, img_pts_right, img_size,
        cam_right, dist_right,
        rvecs_right, tvecs_right);

    std::cout << "[RIGHT] RMS reprojection error: " << rms << "\n";
    std::cout << "[RIGHT] Camera matrix:\n" << cam_right << "\n";
    std::cout << "[RIGHT] Distortion coefficients: " << dist_right << "\n";

    if (rms > 1.0)
        std::cerr << "[RIGHT] WARNING: RMS > 1.0 — consider recapturing calibration images.\n";

    return nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// stereo_calibrate
//
// FIX: The original code averaged per-image R and T matrices directly.
//      This is mathematically wrong for two reasons:
//        1. Averaging rotation matrices destroys orthogonality (the average
//           of two rotation matrices is not a rotation matrix).
//        2. The per-image R/T from individual rvecs/tvecs are relative to
//           the checkerboard, not to each other — you can't just subtract them.
//
//      The correct approach is cv::stereoCalibrate, which performs a full
//      joint bundle adjustment over all image pairs simultaneously, using
//      the pre-computed intrinsics as starting points (CALIB_FIX_INTRINSIC).
//      It directly produces the R and T between the two cameras, plus
//      R1, R2, P1, P2, Q for rectification.
// ─────────────────────────────────────────────────────────────────────────────

void stereo_calibrate() {
    // Require both intrinsic calibrations to have completed
    assert(!cam_left.empty()  && "Left intrinsic calibration not done");
    assert(!cam_right.empty() && "Right intrinsic calibration not done");

    // Image point vectors must have the same number of paired frames
    // (left image i was captured simultaneously with right image i)
    if (img_pts_left.size() != img_pts_right.size()) {
        std::cerr << "[STEREO] Error: left and right have different frame counts ("
                  << img_pts_left.size() << " vs " << img_pts_right.size() << ").\n"
                  << "         Images must be captured in synchronised pairs.\n";
        return;
    }

    const size_t N = img_pts_left.size();
    std::vector<std::vector<cv::Point3f>> obj_pts(N, objp);

    // cv::stereoCalibrate with CALIB_FIX_INTRINSIC:
    //   Keeps cam_left, dist_left, cam_right, dist_right fixed (already optimised)
    //   and only optimises R and T between the cameras.
    //   This is the standard approach when per-camera calibration RMS is < 0.5 px.
    cv::Mat E, F;  // Essential and fundamental matrices (not used downstream but good to have)

    double rms = cv::stereoCalibrate(
        obj_pts,
        img_pts_left,
        img_pts_right,
        cam_left,  dist_left,
        cam_right, dist_right,
        calibration_img_size,
        R_stereo, T_stereo,
        E, F,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                         100, 1e-5));

    std::cout << "[STEREO] RMS reprojection error: " << rms << "\n";
    std::cout << "[STEREO] R (rotation left→right):\n" << R_stereo << "\n";
    std::cout << "[STEREO] T (translation left→right):\n" << T_stereo << "\n";

    if (rms > 1.0)
        std::cerr << "[STEREO] WARNING: RMS > 1.0 — stereo calibration may be inaccurate.\n";

    // Compute baseline (physical distance between cameras in metres)
    baseline = cv::norm(T_stereo);
    std::cout << "[STEREO] Baseline: " << baseline << " m\n";

    // ── Stereo rectification ─────────────────────────────────────────────────
    // Computes R1, R2, P1, P2, Q:
    //   R1, R2 — rotation to apply to each camera image to make epipolar lines horizontal
    //   P1, P2 — projection matrices in the rectified coordinate system
    //   Q      — disparity-to-depth reprojection matrix (used by reprojectImageTo3D)
    // CALIB_ZERO_DISPARITY aligns principal points horizontally (recommended).
    cv::stereoRectify(
        cam_left,  dist_left,
        cam_right, dist_right,
        calibration_img_size,
        R_stereo, T_stereo,
        R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY,
        0,                        // alpha=0: no black borders in rectified images
        calibration_img_size);

    std::cout << "[STEREO] Rectification complete.\n";
    std::cout << "[STEREO] P1 (left  rectified projection):\n" << P1 << "\n";
    std::cout << "[STEREO] P2 (right rectified projection):\n" << P2 << "\n";
    std::cout << "[STEREO] Q  (disparity-to-depth matrix):\n"  << Q  << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// save_extrinsics
//
// Writes all calibration results to extrinsics.txt as a YAML FileStorage.
// The depth pipeline reads this file with load_extrinsics().
//
// Keys written:
//   cam_left, dist_left         — left intrinsics
//   cam_right, dist_right       — right intrinsics
//   R_stereo, T_stereo          — rotation + translation between cameras
//   R1, R2                      — per-camera rectification rotations
//   P1, P2                      — rectified projection matrices
//   Q                           — disparity-to-depth matrix
//   baseline                    — |T_stereo| in metres
//   image_width, image_height   — calibration image size
// ─────────────────────────────────────────────────────────────────────────────

bool save_extrinsics(const std::string& path) {
    if (cam_left.empty() || cam_right.empty() || R_stereo.empty()) {
        std::cerr << "[SAVE] Cannot save — calibration not complete.\n";
        return false;
    }

    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "[SAVE] Cannot open file for writing: " << path << "\n";
        return false;
    }

    // Per-camera intrinsics
    fs << "cam_left"   << cam_left;
    fs << "dist_left"  << dist_left;
    fs << "cam_right"  << cam_right;
    fs << "dist_right" << dist_right;

    // Stereo extrinsics
    fs << "R_stereo"   << R_stereo;
    fs << "T_stereo"   << T_stereo;

    // Rectification outputs
    fs << "R1"         << R1;
    fs << "R2"         << R2;
    fs << "P1"         << P1;
    fs << "P2"         << P2;
    fs << "Q"          << Q;

    // Scalar values
    fs << "baseline"     << baseline;
    fs << "image_width"  << calibration_img_size.width;
    fs << "image_height" << calibration_img_size.height;

    fs.release();

    std::cout << "[SAVE] Calibration saved to: " << path << "\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// load_extrinsics
//
// Reads calibration results back from extrinsics.txt.
// Call this at the start of the depth pipeline instead of running calibration.
// ─────────────────────────────────────────────────────────────────────────────

bool load_extrinsics(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[LOAD] Cannot open calibration file: " << path << "\n";
        return false;
    }

    fs["cam_left"]    >> cam_left;
    fs["dist_left"]   >> dist_left;
    fs["cam_right"]   >> cam_right;
    fs["dist_right"]  >> dist_right;
    fs["R_stereo"]    >> R_stereo;
    fs["T_stereo"]    >> T_stereo;
    fs["R1"]          >> R1;
    fs["R2"]          >> R2;
    fs["P1"]          >> P1;
    fs["P2"]          >> P2;
    fs["Q"]           >> Q;
    fs["baseline"]    >> baseline;

    int w = 0, h = 0;
    fs["image_width"]  >> w;
    fs["image_height"] >> h;
    calibration_img_size = cv::Size(w, h);

    fs.release();

    if (cam_left.empty() || cam_right.empty() || R_stereo.empty()) {
        std::cerr << "[LOAD] File loaded but required fields are missing.\n";
        return false;
    }

    std::cout << "[LOAD] Calibration loaded from: " << path << "\n";
    std::cout << "[LOAD] Baseline: " << baseline << " m\n";
    std::cout << "[LOAD] Image size: " << calibration_img_size << "\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_rectification_maps
//
// Precomputes the remap lookup tables from the loaded/computed calibration.
// Call ONCE after load_extrinsics() or stereo_calibrate().
// Apply with cv::remap(frame, rectified, map1, map2, cv::INTER_LINEAR).
// ─────────────────────────────────────────────────────────────────────────────

void compute_rectification_maps(
    cv::Mat& map1L, cv::Mat& map2L,
    cv::Mat& map1R, cv::Mat& map2R)
{
    assert(!R1.empty() && "Call stereo_calibrate() or load_extrinsics() first");

    cv::initUndistortRectifyMap(
        cam_left,  dist_left,  R1, P1,
        calibration_img_size, CV_32FC1, map1L, map2L);

    cv::initUndistortRectifyMap(
        cam_right, dist_right, R2, P2,
        calibration_img_size, CV_32FC1, map1R, map2R);

    std::cout << "[RECTIFY] Remap tables computed for "
              << calibration_img_size << " images.\n";
}