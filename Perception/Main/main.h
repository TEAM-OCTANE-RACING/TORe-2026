#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "Cone/cone.h"
#include "Keypoints/keypoints.h"

inline constexpr float    CONE_CONF_THRESHOLD = 0.6f;
inline constexpr float    CONF_THRESHOLD      = 0.6f;
inline constexpr float    NMS_THRESHOLD       = 0.45f;
inline constexpr float    NUM_THRESHOLD       = 0.5f;
inline constexpr float    KPT_THRESHOLD       = 0.5f;
inline const cv::Size     MODEL_SIZE          = cv::Size(640, 640);

inline const std::string cone_path      = "Model/cone_best.onnx";
inline const std::string keypoints_path = "/home/slateboi/Documents/Octane/DV/Perception/Model/keypoints_best.onnx";
inline const std::string img_path       = "Images/img3.png";