#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

struct Detection {
    int         class_id;
    float       confidence;
    cv::Rect    box;
    std::string label;
};

inline const std::vector<std::string> CLASS_NAMES = {
    "yellow", "blue", "orange", "large_orange", "unknown"
};
inline const int NUM_CLASSES  = (int)CLASS_NAMES.size();
inline const int NUM_FEATURES = 4 + NUM_CLASSES;

cv::dnn::Net           load_net_cone(const std::string& path, bool use_cuda);
std::vector<Detection> detect(const cv::Mat& img, cv::dnn::Net& net);
void                   draw_detections(cv::Mat& img, const std::vector<Detection>& dets);