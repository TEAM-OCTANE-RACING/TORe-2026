#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>


#include "keypoints.h"
#include "Cone/cone.h"
#include "Main/main.h"


const std::vector<cv::Scalar> KEYPOINT_COLORS = {
    cv::Scalar(0, 255, 255),   cv::Scalar(0, 255, 255),   // top    - Yellow
    cv::Scalar(0, 165, 255),   cv::Scalar(0, 165, 255),   // upper  - Orange
    cv::Scalar(0, 0, 255),     cv::Scalar(0, 0, 255),     // lower  - Red
    cv::Scalar(255, 0, 0),     cv::Scalar(255, 0, 0),     // bottom - Blue
};



cv::dnn::Net load_net_keypoints(const std::string &path, bool use_cuda){

    std::cout << "Loading ONNX: " << path << std::endl;

    cv::dnn::Net net = cv::dnn::readNetFromONNX(path);


    if (net.empty()){
        std::cerr << "[ERROR] Failed to load model: " << path << "\n";
        exit(EXIT_FAILURE);
    }

    if(use_cuda){
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    else{
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    return net;
}

cv::Scalar get_keypoint_color(int keypoint_index) {
    if (keypoint_index < 0 || keypoint_index >= (int)KEYPOINT_COLORS.size())
        return cv::Scalar(255, 255, 255);
    return KEYPOINT_COLORS[keypoint_index];
}

void draw_keypoints(cv::Mat& img, const Keypoints& kpts){
    for (int k = 0; k < 8; k++) {
        if (kpts.p[k].visibility < KPT_THRESHOLD) continue;

        cv::Scalar color = get_keypoint_color(k);
        cv::circle(img, kpts.p[k].point, 5, color,              -1, cv::LINE_AA);
        cv::circle(img, kpts.p[k].point, 6, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
}

void detect_keypoints(std::vector<Detection>& detections, cv::Mat& img, cv::dnn::Net& net){
    for(auto& detection : detections){

        cv::Mat roi = img(detection.box).clone();

        cv::Mat blob;
        cv::dnn::blobFromImage(roi, blob, 1.0 / 255.0, MODEL_SIZE, cv::Scalar(), true, false);
        net.setInput(blob);

        std::vector<cv::Mat> outputs;
        net.forward(outputs, net.getUnconnectedOutLayersNames());

        int rows = outputs[0].size[2];
        int dimensions = outputs[0].size[1];
        cv::Mat out = outputs[0].reshape(1, dimensions);
        cv::transpose(out, out);
        float* data = (float*)out.data;

        float x_scale = (float)detection.box.width  / MODEL_SIZE.width;
        float y_scale = (float)detection.box.height / MODEL_SIZE.height;

        Keypoints best;
        best.confidence = -1.0f;

        for(int i = 0; i < rows; i++, data += dimensions){
            float obj_conf = data[4];
            if (obj_conf < CONF_THRESHOLD) continue;
            if (obj_conf <= best.confidence) continue;

            best.confidence = obj_conf;

            for(int k = 0; k < 8; k++){
                int base = 5 + k * 3;
                best.p[k].point.x   = data[base + 0] * x_scale + detection.box.x;
                best.p[k].point.y   = data[base + 1] * y_scale + detection.box.y;
                best.p[k].visibility = data[base + 2];
            }
        }

        if(best.confidence > 0.0f){
            draw_keypoints(img, best);
        }
    }
}

