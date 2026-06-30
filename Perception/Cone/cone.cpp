
//  Pre-defined headers
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

//  User-defined headers
#include "cone.h"
#include "Main/main.h"
#include "Keypoints/keypoints.h"


cv::dnn::Net load_net_cone(const std::string& path, bool use_cuda){
    cv::dnn::Net net = cv::dnn::readNetFromONNX(path);
    if (net.empty()) {
        std::cerr << "[ERROR] Failed to load model: " << path << "\n";
        exit(EXIT_FAILURE);
    }
    if (use_cuda) {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        //std::cout << "[INFO] Backend: CUDA\n";
    } else {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        //std::cout << "[INFO] Backend: CPU\n";
    }
    return net;
}

// ─── Detect ───────────────────────────────────────────────────────────────────
// Ultralytics YOLOv8 Python export → ONNX output shape: [1, NUM_FEATURES, 8400]
// Layout: [1, 8, 8400] for 4-class model
// After reshape(1, NUM_FEATURES) + transpose → [8400, NUM_FEATURES]
// Each row: [cx, cy, w, h, score_0, score_1, score_2, score_3]
// Coordinates are in model pixel space: 0 → 640

std::vector<Detection> detect(const cv::Mat& img, cv::dnn::Net& net){

    // ── Preprocess ────────────────────────────────────────────────────────────
    cv::Mat blob;
    cv::dnn::blobFromImage(img, blob, 1.0 / 255.0, MODEL_SIZE, cv::Scalar(), true, false);
    net.setInput(blob);

    // ── Forward pass ──────────────────────────────────────────────────────────
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    cv::Mat& raw = outputs[0];   // shape: [1, NUM_FEATURES, 8400]

    // ── Validate tensor shape ─────────────────────────────────────────────────
    if (raw.dims != 3) {
        std::cerr << "[ERROR] Expected 3D tensor, got " << raw.dims << "D\n";
        return {};
    }

    const int d1 = raw.size[1];   // should be NUM_FEATURES (8)
    const int d2 = raw.size[2];   // should be 8400

    std::cout << "\n\n\n\n\n\n" << d1 << d2 << std::endl;

    std::cout << "[DEBUG] Tensor: 1 x " << d1 << " x " << d2 << "\n";

    if (d1 != NUM_FEATURES) {
        std::cerr << "[ERROR] Expected dim[1]=" << NUM_FEATURES
                  << " but got " << d1
                  << ". Check that config has exactly " << NUM_CLASSES
                  << " classes and matches training order.\n";
        return {};
    }

    const int num_anchors = d2;   // 8400

    // ── Reshape [1, 8, 8400] → [8, 8400] → transpose → [8400, 8] ─────────────
    cv::Mat out = raw.reshape(1, NUM_FEATURES);  // [8, 8400]
    cv::transpose(out, out);                      // [8400, 8]

    // ── Scale from model space (0–640) to original image space ────────────────
    const float x_scale = (float)img.cols / MODEL_SIZE.width;
    const float y_scale = (float)img.rows / MODEL_SIZE.height;

    // ── Debug: print first 3 anchors to verify sane values ───────────────────
    std::cout << "[DEBUG] Sample anchors (cx cy w h | scores):\n";
    for (int i = 0; i < std::min(3, num_anchors); i++) {
        float* r = (float*)out.ptr(i);
        std::cout << "  [" << i << "] cx=" << r[0] << " cy=" << r[1]
                  << " w=" << r[2] << " h=" << r[3] << " | ";
        for (int j = 4; j < NUM_FEATURES; j++)
            std::cout << CLASS_NAMES[j-4] << "=" << r[j] << " ";
        std::cout << "\n";
    }

    std::vector<int>      class_ids;
    std::vector<float>    confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < num_anchors; i++) {
        float* row = (float*)out.ptr(i);  // [cx, cy, w, h, s0, s1, s2, s3]

        // Find best class score
        float best_score = -1.0f;
        int   best_cls   = 0;
        for (int c = 0; c < NUM_CLASSES; c++) {
            if (row[4 + c] > best_score) {
                best_score = row[4 + c];
                best_cls   = c;
            }
        }

        if (best_score < CONE_CONF_THRESHOLD) continue;

        float cx = row[0], cy = row[1];
        float bw = row[2], bh = row[3];

        if (bw <= 0 || bh <= 0) continue;

        int left   = (int)((cx - bw * 0.5f) * x_scale);
        int top    = (int)((cy - bh * 0.5f) * y_scale);
        int width  = (int)(bw * x_scale);
        int height = (int)(bh * y_scale);

        // Clamp to image bounds
        left   = std::max(0, left);
        top    = std::max(0, top);
        width  = std::min(width,  img.cols - left);
        height = std::min(height, img.rows - top);

        if (width < 2 || height < 2) continue;

        class_ids.push_back(best_cls);
        confidences.push_back(best_score);
        boxes.push_back(cv::Rect(left, top, width, height));
    }

    std::cout << "[INFO] Boxes before NMS: " << boxes.size() << "\n";

    // ── NMS ───────────────────────────────────────────────────────────────────
    std::vector<int> nms_idx;
    cv::dnn::NMSBoxes(boxes, confidences, CONE_CONF_THRESHOLD, NMS_THRESHOLD, nms_idx);

    std::vector<Detection> detections;
    detections.reserve(nms_idx.size());
    for (int idx : nms_idx) {
        Detection d;
        d.class_id   = class_ids[idx];
        d.confidence = confidences[idx];
        d.box        = boxes[idx];
        d.label      = CLASS_NAMES[d.class_id];
        detections.push_back(d);
    }

    std::cout << "[INFO] Detections after NMS: " << detections.size() << "\n";
    return detections;
}

// ─── Draw ─────────────────────────────────────────────────────────────────────
void draw_detections(cv::Mat& img, const std::vector<Detection>& dets){
    // One colour per class: yellow, blue, orange, large_orange
    const std::vector<cv::Scalar> COLOURS = {
        {0,   255, 255},   // yellow cone  → yellow box
        {255, 128,   0},   // blue cone    → blue box
        {0,   128, 255},   // orange cone  → orange box
        {0,   0,   200},   // large_orange → dark red box
    };

    for (const auto& d : dets) {
        cv::Scalar col = COLOURS[d.class_id % COLOURS.size()];
        cv::rectangle(img, d.box, col, 2);

        std::string text = d.label + " " + std::to_string((int)(d.confidence * 100)) + "%";
        int baseline = 0;
        cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

        cv::Rect bg(d.box.x, d.box.y - ts.height - 8,
                    ts.width + 4, ts.height + 8);
        bg &= cv::Rect(0, 0, img.cols, img.rows);

        cv::rectangle(img, bg, col, cv::FILLED);
        cv::putText(img, text, cv::Point(d.box.x + 2, d.box.y - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────

/*
int main() {
    std::cout << "[INFO] Classes: ";
    for (const auto& c : CLASS_NAMES) std::cout << c << " ";
    std::cout << "\n";

    cv::Mat img = cv::imread(IMAGE_PATH, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "[ERROR] Cannot read image: " << IMAGE_PATH << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "[INFO] Image: " << img.cols << "x" << img.rows << "\n";

    cv::dnn::Net net = load_net(MODEL_PATH, USE_CUDA);
    auto dets = detect(img, net);
    draw_detections(img, dets);

    cv::imshow("Detections", img);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}

*/