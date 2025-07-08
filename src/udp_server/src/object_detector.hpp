#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <string>

struct DetectedObject {
    float x, y, z;  // 3D coordinates in meters (camera space)
    std::string type;
    float confidence;
    
    DetectedObject(float x_, float y_, float z_, const std::string& type_, float conf_)
        : x(x_), y(y_), z(z_), type(type_), confidence(conf_) {}
};

struct DetectionResult {
    std::vector<DetectedObject> hands;
    std::vector<DetectedObject> objects;
    uint32_t timestamp;
    
    DetectionResult(uint32_t ts) : timestamp(ts) {}
};

class ObjectDetector {
public:
    ObjectDetector();
    ~ObjectDetector() = default;
    
    // Main detection function
    DetectionResult detectObjects(const cv::Mat& rgb, const cv::Mat& depth, uint32_t timestamp);
    
    // Configuration
    void setHandDetectionEnabled(bool enabled) { detect_hands_ = enabled; }
    void setObjectDetectionEnabled(bool enabled) { detect_objects_ = enabled; }
    void setMinHandArea(int area) { min_hand_area_ = area; }
    void setMaxHandArea(int area) { max_hand_area_ = area; }
    void setMinObjectArea(int area) { min_object_area_ = area; }
    void setDepthRange(float min_depth, float max_depth) { 
        min_depth_ = min_depth; 
        max_depth_ = max_depth; 
    }

private:
    // Hand detection
    std::vector<DetectedObject> detectHands(const cv::Mat& rgb, const cv::Mat& depth);
    cv::Mat createSkinMask(const cv::Mat& rgb);
    
    // Object detection  
    std::vector<DetectedObject> detectGenericObjects(const cv::Mat& depth);
    cv::Mat createDepthMask(const cv::Mat& depth);
    
    // Utility functions
    cv::Point3f pixelToWorldCoordinates(int x, int y, float depth_mm);
    bool isValidDepth(float depth_mm);
    float calculateConfidence(const cv::Mat& mask, const std::vector<cv::Point>& contour);
    
    // Configuration parameters
    bool detect_hands_ = true;
    bool detect_objects_ = true;
    
    // Hand detection parameters
    int min_hand_area_ = 1000;
    int max_hand_area_ = 15000;
    cv::Scalar lower_skin_hsv_ = cv::Scalar(0, 48, 80);
    cv::Scalar upper_skin_hsv_ = cv::Scalar(20, 255, 255);
    
    // Object detection parameters
    int min_object_area_ = 2000;
    float min_depth_ = 500.0f;   // mm
    float max_depth_ = 4000.0f;  // mm
    
    // Camera intrinsic parameters (Kinect defaults)
    float fx_ = 594.21f;  // focal length x
    float fy_ = 591.04f;  // focal length y  
    float cx_ = 339.5f;   // principal point x
    float cy_ = 242.7f;   // principal point y
};