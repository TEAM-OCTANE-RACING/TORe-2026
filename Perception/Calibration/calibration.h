#pragma once

/*
 * calibration.h
 *
 * Stereo camera calibration — intrinsic and extrinsic.
 * Results saved to / loaded from extrinsics.txt (YAML format).
 *
 * Typical usage flow:
 *
 *   ONE-TIME (run once, then hardcode or load from file):
 *     prepare_objp();
 *     pthread_t tL, tR;
 *     pthread_create(&tL, nullptr, left_intrinsic_calibrate,  nullptr);
 *     pthread_create(&tR, nullptr, right_intrinsic_calibrate, nullptr);
 *     pthread_join(tL, nullptr);
 *     pthread_join(tR, nullptr);
 *     stereo_calibrate();
 *     save_extrinsics("extrinsics.txt");
 *
 *   PER-SESSION (load saved calibration, skip re-running):
 *     load_extrinsics("extrinsics.txt");
 *     cv::Mat map1L, map2L, map1R, map2R;
 *     compute_rectification_maps(map1L, map2L, map1R, map2R);
 *     // Then apply with cv::remap() before passing to depth pipeline
 */

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// Checkerboard parameters — set to match your physical board
// ─────────────────────────────────────────────────────────────────────────────

#define ROW                7       // inner corner rows
#define COL                11      // inner corner columns
#define CHECKERBOARD_SIZE  0.03f   // square side length in metres

// ─────────────────────────────────────────────────────────────────────────────
// Calibration results — extern globals accessible by other translation units
// ─────────────────────────────────────────────────────────────────────────────

// Per-camera intrinsics (from left/right_intrinsic_calibrate)
extern cv::Mat cam_left,  dist_left;
extern cv::Mat cam_right, dist_right;

// Per-frame rotation/translation vectors from intrinsic calibration
extern std::vector<cv::Mat> rvecs_left,  tvecs_left;
extern std::vector<cv::Mat> rvecs_right, tvecs_right;

// Stereo extrinsics (from stereo_calibrate)
extern cv::Mat R_stereo;   // 3x3 rotation:    right = R_stereo * left
extern cv::Mat T_stereo;   // 3x1 translation: right = R_stereo * left + T_stereo

// Rectification outputs (from cv::stereoRectify inside stereo_calibrate)
extern cv::Mat R1, R2;     // per-camera rectification rotations
extern cv::Mat P1, P2;     // rectified projection matrices
                           //   fx = P1[0][0], fy = P1[1][1]
                           //   cx = P1[0][2], cy = P1[1][2]
extern cv::Mat Q;          // disparity-to-depth reprojection matrix

// Scalar calibration values
extern double  baseline;              // |T_stereo| in metres
extern cv::Size calibration_img_size; // size of images used during calibration

// Image corner point collections (needed by stereo_calibrate — filled during intrinsic calibration)
extern std::vector<std::vector<cv::Point2f>> img_pts_left;
extern std::vector<std::vector<cv::Point2f>> img_pts_right;

// Object points in 3D (one row of the checkerboard pattern)
extern std::vector<cv::Point3f> objp;

// ─────────────────────────────────────────────────────────────────────────────
// Function declarations
// ─────────────────────────────────────────────────────────────────────────────

/**
 * prepare_objp
 * Fills the global objp vector with 3D checkerboard corner positions.
 * Must be called ONCE before starting calibration threads.
 * Thread-safe if called before threads are spawned.
 */
void prepare_objp();

/**
 * left_intrinsic_calibrate
 * pthread-compatible function. Reads images from ./leftcamera/*.png,
 * detects chessboard corners, runs cv::calibrateCamera, fills:
 *   cam_left, dist_left, rvecs_left, tvecs_left, img_pts_left
 */
void* left_intrinsic_calibrate(void* arg);

/**
 * right_intrinsic_calibrate
 * pthread-compatible function. Reads images from ./rightcamera/*.png,
 * detects chessboard corners, runs cv::calibrateCamera, fills:
 *   cam_right, dist_right, rvecs_right, tvecs_right, img_pts_right
 */
void* right_intrinsic_calibrate(void* arg);

/**
 * stereo_calibrate
 * Runs cv::stereoCalibrate (CALIB_FIX_INTRINSIC) on paired image points,
 * then cv::stereoRectify. Fills:
 *   R_stereo, T_stereo, R1, R2, P1, P2, Q, baseline
 *
 * Requires: left_intrinsic_calibrate and right_intrinsic_calibrate
 *           must both have completed successfully first.
 */
void stereo_calibrate();

/**
 * save_extrinsics
 * Writes all calibration results to a YAML file at the given path.
 * Default path: "extrinsics.txt"
 * Returns true on success.
 */
bool save_extrinsics(const std::string& path = "extrinsics.txt");

/**
 * load_extrinsics
 * Reads calibration results from a YAML file written by save_extrinsics().
 * Fills all global calibration variables.
 * Returns true on success.
 * Call this instead of running calibration if you have a saved file.
 */
bool load_extrinsics(const std::string& path = "extrinsics.txt");

/**
 * compute_rectification_maps
 * Precomputes cv::remap lookup tables from R1/R2/P1/P2.
 * Call once after stereo_calibrate() or load_extrinsics().
 *
 * Usage:
 *   cv::Mat map1L, map2L, map1R, map2R;
 *   compute_rectification_maps(map1L, map2L, map1R, map2R);
 *
 *   // Every frame:
 *   cv::remap(frameL, rectL, map1L, map2L, cv::INTER_LINEAR);
 *   cv::remap(frameR, rectR, map1R, map2R, cv::INTER_LINEAR);
 */
void compute_rectification_maps(
    cv::Mat& map1L, cv::Mat& map2L,
    cv::Mat& map1R, cv::Mat& map2R);