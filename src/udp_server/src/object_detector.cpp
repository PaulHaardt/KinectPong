#include "object_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <iostream>

ObjectDetector::ObjectDetector() {
    // Constructor - can load calibration data here if available
}

DetectionResult ObjectDetector::detectObjects(const cv::Mat& rgb, const cv::Mat& depth, uint32_t timestamp) {
    DetectionResult result(timestamp);
    
    if (rgb.empty() || depth.empty()) {
        return result;
    }
    
    if (detect_hands_) {
        result.hands = detectHands(rgb, depth);
    }
    
    if (detect_objects_) {
        result.objects = detectGenericObjects(depth);
    }
    
    return result;
}

std::vector<DetectedObject> ObjectDetector::detectHands(const cv::Mat& rgb, const cv::Mat& depth) {
    std::vector<DetectedObject> hands;
    
    // Create skin color mask
    cv::Mat skin_mask = createSkinMask(rgb);
    
    // Create depth mask for hand distance range
    cv::Mat depth_mask = createDepthMask(depth);
    
    // Combine masks
    cv::Mat combined_mask;
    cv::bitwise_and(skin_mask, depth_mask, combined_mask);
    
    // Morphological operations to clean up the mask
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_CLOSE, kernel);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(combined_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        
        if (area >= min_hand_area_ && area <= max_hand_area_) {
            // Calculate centroid
            cv::Moments moments = cv::moments(contour);
            if (moments.m00 > 0) {
                int cx = static_cast<int>(moments.m10 / moments.m00);
                int cy = static_cast<int>(moments.m01 / moments.m00);
                
                // Get depth at centroid
                if (cx >= 0 && cx < depth.cols && cy >= 0 && cy < depth.rows) {
                    uint16_t depth_val = depth.at<uint16_t>(cy, cx);
                    
                    if (isValidDepth(static_cast<float>(depth_val))) {
                        // Convert to world coordinates
                        cv::Point3f world_pos = pixelToWorldCoordinates(cx, cy, static_cast<float>(depth_val));
                        
                        // Calculate confidence based on area and mask quality
                        float confidence = calculateConfidence(combined_mask, contour);
                        
                        hands.emplace_back(world_pos.x, world_pos.y, world_pos.z, "hand", confidence);
                    }
                }
            }
        }
    }
    
    return hands;
}

std::vector<DetectedObject> ObjectDetector::detectGenericObjects(const cv::Mat& depth) {
    std::vector<DetectedObject> objects;
    
    // Create depth mask for object detection range
    cv::Mat depth_mask = createDepthMask(depth);
    
    // Apply morphological operations to reduce noise
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_CLOSE, kernel);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(depth_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        
        if (area >= min_object_area_) {
            // Calculate centroid
            cv::Moments moments = cv::moments(contour);
            if (moments.m00 > 0) {
                int cx = static_cast<int>(moments.m10 / moments.m00);
                int cy = static_cast<int>(moments.m01 / moments.m00);
                
                // Get depth at centroid
                if (cx >= 0 && cx < depth.cols && cy >= 0 && cy < depth.rows) {
                    uint16_t depth_val = depth.at<uint16_t>(cy, cx);
                    
                    if (isValidDepth(static_cast<float>(depth_val))) {
                        // Convert to world coordinates
                        cv::Point3f world_pos = pixelToWorldCoordinates(cx, cy, static_cast<float>(depth_val));
                        
                        // Calculate confidence based on area and compactness
                        float confidence = calculateConfidence(depth_mask, contour);
                        
                        objects.emplace_back(world_pos.x, world_pos.y, world_pos.z, "object", confidence);
                    }
                }
            }
        }
    }
    
    return objects;
}

cv::Mat ObjectDetector::createSkinMask(const cv::Mat& rgb) {
    cv::Mat hsv, mask;
    cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
    
    // Create mask for skin color range
    cv::inRange(hsv, lower_skin_hsv_, upper_skin_hsv_, mask);
    
    // Additional skin color range (for different lighting conditions)
    cv::Mat mask2;
    cv::inRange(hsv, cv::Scalar(160, 48, 80), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask, mask2, mask);
    
    return mask;
}

cv::Mat ObjectDetector::createDepthMask(const cv::Mat& depth) {
    cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8UC1);
    
    depth.forEach<uint16_t>([&](uint16_t& pixel, const int position[2]) {
        float depth_mm = static_cast<float>(pixel);
        if (isValidDepth(depth_mm) && depth_mm >= min_depth_ && depth_mm <= max_depth_) {
            mask.at<uint8_t>(position[0], position[1]) = 255;
        }
    });
    
    return mask;
}

cv::Point3f ObjectDetector::pixelToWorldCoordinates(int x, int y, float depth_mm) {
    // Convert depth from mm to meters
    float z = depth_mm / 1000.0f;
    
    // Convert pixel coordinates to world coordinates using camera intrinsics
    float world_x = (x - cx_) * z / fx_;
    float world_y = (y - cy_) * z / fy_;
    
    return cv::Point3f(world_x, world_y, z);
}

bool ObjectDetector::isValidDepth(float depth_mm) {
    return depth_mm > 0 && depth_mm < 10000; // Valid Kinect range
}

float ObjectDetector::calculateConfidence(const cv::Mat& mask, const std::vector<cv::Point>& contour) {
    // Calculate confidence based on contour properties
    double area = cv::contourArea(contour);
    double perimeter = cv::arcLength(contour, true);
    
    if (perimeter == 0) return 0.0f;
    
    // Compactness measure (closer to 1.0 = more circular/compact)
    float compactness = static_cast<float>(4 * CV_PI * area / (perimeter * perimeter));
    
    // Normalize and clamp confidence
    float confidence = std::min(1.0f, std::max(0.1f, compactness));
    
    return confidence;
}