#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "Cone/cone.h"

struct Point {
    cv::Point2f point;
    float       visibility;
};

struct Keypoints {
    float confidence;
    Point p[8];
};

cv::dnn::Net  load_net_keypoints(const std::string& path, bool use_cuda);
void          draw_keypoints(cv::Mat& img, const Keypoints& kpts);
void          detect_keypoints(std::vector<Detection>& detections, cv::Mat& img, cv::dnn::Net& net);