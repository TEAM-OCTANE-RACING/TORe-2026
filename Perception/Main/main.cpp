/*
 * main.cpp
 *
 * Execution order per frame:
 *   1. Load extrinsics from extrinsics.txt (once at startup)
 *   2. Build rectification maps              (once at startup)
 *   3. Rectify stereo image pair             (every frame)
 *   4. YOLO cone detection on rectified left (every frame)
 *   5. YOLO keypoint detection               (every frame)
 *   6. Stereo depth + bearing estimation     (every frame)
 *   7. Visualise and display                 (every frame)
 */

// Pre-defined headers
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>

// User-defined headers
#include "Cone/cone.h"
#include "Keypoints/keypoints.h"
#include "Calibration/rectification.h"
#include "Depth/cone_depth.hpp"
#include "main.h"

int main()
{
    // ── 1. Load extrinsics + build rectification maps ─────────────────────────
    //
    // Rectifier reads cam_left, dist_left, cam_right, dist_right,
    // R1, R2, P1, P2 and image size from extrinsics.txt, then
    // precomputes the remap lookup tables internally.
    //
    // CameraConfig reads P1 (fx, fy, cx, cy) and baseline from the same file.
    // Both reads are independent — no shared globals between them.

    Rectifier rectifier;
    if (!rectifier.init_from_file("extrinsics.txt")) {
        std::cerr << "[MAIN] Failed to load rectification data from extrinsics.txt\n"
                  << "       Run calibration first and save via save_extrinsics().\n";
        return 1;
    }

    ConeDepth::CameraConfig cam;
    if (!cam.load_from_file("extrinsics.txt")) {
        std::cerr << "[MAIN] Failed to load camera intrinsics from extrinsics.txt\n";
        return 1;
    }

    // ── 2. Pipeline configuration ─────────────────────────────────────────────
    //
    // disp_max: maximum disparity to search.
    // Formula: fx * baseline / min_expected_cone_distance
    // Example: 700 * 0.12 / 0.5m = 168 → 200 with margin.
    // Adjust min_cone_dist_m to your track's closest expected cone.

    ConeDepth::PipelineConfig depth_cfg;
    const double min_cone_dist_m = 0.5;
    depth_cfg.disp_max = static_cast<int>(
        std::ceil(cam.fx * cam.baseline / min_cone_dist_m)) + 30;

    std::cout << "[MAIN] disp_max set to " << depth_cfg.disp_max
              << "  (fx=" << cam.fx
              << " B="    << cam.baseline
              << " min_dist=" << min_cone_dist_m << "m)\n";

    // ── 3. Load YOLO models ───────────────────────────────────────────────────

    cv::dnn::Net cone_net      = load_net_cone(cone_path,      false);
    cv::dnn::Net keypoints_net = load_net_keypoints(keypoints_path, false);

    // ── 4. Load image(s) ─────────────────────────────────────────────────────
    //
    // For a live stereo camera replace these two imread calls with
    // a VideoCapture grab + retrieve pair.
    //
    // img_path_left and img_path_right must be defined in main.h.
    // Both images must be the same size as used during calibration.

    cv::Mat raw_left  = cv::imread(img_path_left,  cv::IMREAD_COLOR);
    cv::Mat raw_right = cv::imread(img_path_right, cv::IMREAD_COLOR);

    if (raw_left.empty() || raw_right.empty()) {
        std::cerr << "[MAIN] Failed to load images.\n"
                  << "       img_path_left  = " << img_path_left  << "\n"
                  << "       img_path_right = " << img_path_right << "\n";
        return 1;
    }

    // ── 5. Rectify both images ────────────────────────────────────────────────
    //
    // rectify() applies cv::remap with the precomputed lookup tables.
    // After this call:
    //   - Lens distortion is removed
    //   - Epipolar lines are horizontal (y_right = y_left for any match)
    // All subsequent processing uses rect_left and rect_right — never the raws.

    cv::Mat rect_left, rect_right;
    if (!rectifier.rectify(raw_left, raw_right, rect_left, rect_right)) {
        std::cerr << "[MAIN] Rectification failed. "
                     "Check that image size matches calibration.\n";
        return 1;
    }

    // Optional: draw horizontal epipolar lines to visually verify alignment.
    // Features on the same horizontal line in both images = correct calibration.
    // Uncomment during debugging:
    //
    // cv::Mat epipolar_vis_L = rect_left.clone();
    // cv::Mat epipolar_vis_R = rect_right.clone();
    // Rectifier::draw_epipolar_lines(epipolar_vis_L, epipolar_vis_R, 50);
    // cv::imshow("Epipolar Left",  epipolar_vis_L);
    // cv::imshow("Epipolar Right", epipolar_vis_R);

    // ── 6. Cone detection on rectified left image ─────────────────────────────
    //
    // detect() runs YOLO on the left rectified image.
    // All downstream stages (keypoints, depth) use left-image coordinates.

    std::vector<Detection> detections = detect(rect_left, cone_net);

    // Draw bounding boxes onto a visualisation copy of the left image
    cv::Mat vis_left  = rect_left.clone();
    cv::Mat vis_right = rect_right.clone();
    draw_detections(vis_left, detections);

    // ── 7. Keypoint detection ─────────────────────────────────────────────────
    //
    // detect_keypoints() returns one Keypoints per Detection (parallel vector).
    // Entries where no keypoint detection passed CONF_THRESHOLD have
    // confidence = -1.0f and are skipped by estimate_depths() automatically.
    //
    // Keypoint pixel coordinates are in the full rect_left frame (not ROI-local).

    std::vector<Keypoints> keypoints =
        detect_keypoints(detections, vis_left, keypoints_net);

    // ── 8. Stereo depth + bearing estimation ──────────────────────────────────
    //
    // estimate_depths() consumes:
    //   rect_left, rect_right  — rectified stereo pair
    //   detections             — bounding boxes + class/confidence from YOLO
    //   keypoints              — 8 keypoints per cone from keypoint model
    //   cam                    — fx, fy, cx, cy, baseline from extrinsics.txt
    //   depth_cfg              — NCC parameters, depth range, ring weights
    //
    // Returns one ConeResult per Detection (parallel, same index).
    // ConeResult fields:
    //   .valid       — false if no confident depth could be estimated
    //   .depth_z     — Z in metres along the optical axis
    //   .bearing     — degrees: negative = left of centre, positive = right
    //   .position    — (X, Y, Z) in camera frame (metres)
    //   .valid_rings — how many rings contributed (0–4)
    //   .label       — cone colour: "yellow", "blue", etc.

    std::vector<ConeDepth::ConeResult> depth_results =
        ConeDepth::estimate_depths(
            rect_left, rect_right,
            detections, keypoints,
            cam, depth_cfg);

    // ── 9. Print depth results ────────────────────────────────────────────────

    std::cout << "\n[MAIN] Depth results (" << depth_results.size() << " cones):\n";
    std::cout << std::fixed << std::setprecision(3);

    for (const auto& r : depth_results) {
        if (!r.valid) {
            std::cout << "  cone " << std::setw(2) << r.class_id
                      << " (" << r.label << ")"
                      << "  — no valid depth"
                      << "  [conf=" << r.det_conf << "]\n";
            continue;
        }

        const char* side = r.bearing >= 0.0 ? "RIGHT" : "LEFT";
        std::cout << "  cone " << std::setw(2) << r.class_id
                  << " (" << r.label << ")"
                  << "  Z="      << std::setw(6) << r.depth_z  << "m"
                  << "  X="      << std::setw(6) << r.position.x << "m"
                  << "  Y="      << std::setw(6) << r.position.y << "m"
                  << "  "        << side
                  << " "         << std::setw(5) << std::abs(r.bearing) << "deg"
                  << "  rings="  << r.valid_rings << "/4"
                  << "  conf="   << r.det_conf
                  << "\n";
    }

    // ── 10. Visualise depth on both images ────────────────────────────────────
    //
    // draw_results() annotates:
    //   vis_left:  ring-coloured keypoint dots, disparity labels,
    //              "label  Z=X.XXm  L/R Y.Ydeg  [N/4]" per cone
    //   vis_right: matched keypoint positions at same epipolar row

    ConeDepth::draw_results(vis_left, vis_right, depth_results);

    // ── 11. Display and save ──────────────────────────────────────────────────

    cv::imshow("Left  — depth + bearing", vis_left);
    cv::imshow("Right — matched points",  vis_right);

    if (cv::imwrite("output_left.png",  vis_left))
        std::cout << "[MAIN] Saved output_left.png\n";
    else
        std::cerr << "[MAIN] Failed to save output_left.png\n";

    if (cv::imwrite("output_right.png", vis_right))
        std::cout << "[MAIN] Saved output_right.png\n";
    else
        std::cerr << "[MAIN] Failed to save output_right.png\n";

    cv::waitKey(0);
    return 0;
}