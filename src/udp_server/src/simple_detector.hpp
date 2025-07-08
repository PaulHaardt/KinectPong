#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>

// Reuse the same structures from our main server
struct SimpleDetectedObject {
    float x, y, z;
    std::string type;
    float confidence;
    
    SimpleDetectedObject(float x_, float y_, float z_, const std::string& type_, float conf_)
        : x(x_), y(y_), z(z_), type(type_), confidence(conf_) {}
};

class SimpleDetector {
public:
    SimpleDetector() {
        // Kinect camera intrinsics (from existing object_detector.hpp)
        fx_ = 594.21f;
        fy_ = 591.04f; 
        cx_ = 339.5f;
        cy_ = 242.7f;
        
        // Detection parameters (KISS - from existing code)
        min_hand_area_ = 1000;
        max_hand_area_ = 15000;
        min_object_area_ = 2000;
        
        // Depth ranges (in mm)
        hand_min_depth_ = 500.0f;   // 0.5m
        hand_max_depth_ = 1500.0f;  // 1.5m  
        object_min_depth_ = 800.0f;   // 0.8m (table surface)
        object_max_depth_ = 1200.0f;  // 1.2m
        
        // Skin color HSV range (from existing code)
        lower_skin_hsv_ = cv::Scalar(0, 48, 80);
        upper_skin_hsv_ = cv::Scalar(20, 255, 255);
        
        std::cout << "SimpleDetector initialized with KISS parameters" << std::endl;
    }
    
    // Main detection function
    std::pair<std::vector<SimpleDetectedObject>, std::vector<SimpleDetectedObject>> 
    detectObjects(const cv::Mat& rgb, const cv::Mat& depth) {
        
        std::vector<SimpleDetectedObject> hands;
        std::vector<SimpleDetectedObject> objects;
        
        // Handle empty frames gracefully (dummy mode or initialization)
        if (rgb.empty() || depth.empty()) {
            return {hands, objects};
        }
        
        // DEBUG: Log frame info periodically
        static int debug_frame_count = 0;
        debug_frame_count++;
        bool debug_this_frame = (debug_frame_count % 120 == 1);  // Every 4 seconds
        
        if (debug_this_frame) {
            std::cout << "[DETECTOR] Processing frame - RGB: " << rgb.size() 
                      << " Depth: " << depth.size() << std::endl;
        }
        
        // Ensure correct formats
        if (rgb.channels() != 3 || depth.type() != CV_16UC1) {
            if (debug_this_frame) {
                std::cout << "[DETECTOR] Warning: Unexpected frame format - RGB channels: " 
                          << rgb.channels() << " Depth type: " << depth.type() << std::endl;
            }
            return {hands, objects};
        }
        
        try {
            // Check if this is real data or dummy data
            cv::Scalar rgb_mean = cv::mean(rgb);
            cv::Scalar depth_mean = cv::mean(depth);
            bool is_real_data = (rgb_mean[0] > 1 || rgb_mean[1] > 1 || rgb_mean[2] > 1 || depth_mean[0] > 1);
            
            if (debug_this_frame) {
                std::cout << "[DETECTOR] RGB mean: (" << rgb_mean[0] << "," << rgb_mean[1] << "," << rgb_mean[2] 
                          << ") Depth mean: " << depth_mean[0] << " Real data: " << is_real_data << std::endl;
            }
            
            if (!is_real_data) {
                // This is dummy data (all zeros) - return empty results
                if (debug_this_frame) {
                    std::cout << "[DETECTOR] Dummy data detected - returning empty results" << std::endl;
                }
                return {hands, objects};
            }
            
            // Detect hands using skin color + depth
            hands = detectHands(rgb, depth, debug_this_frame);
            
            // Detect objects using depth thresholding  
            objects = detectTableObjects(depth, debug_this_frame);
            
            if (debug_this_frame) {
                std::cout << "[DETECTOR] Detection complete - " << hands.size() 
                          << " hands, " << objects.size() << " objects" << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "[DETECTOR] Detection error: " << e.what() << std::endl;
        }
        
        return {hands, objects};
    }

private:
    
    std::vector<SimpleDetectedObject> detectHands(const cv::Mat& rgb, const cv::Mat& depth, bool debug = false) {
        std::vector<SimpleDetectedObject> hands;
        
        // Step 1: Create skin color mask
        cv::Mat skin_mask = createSkinMask(rgb, debug);
        
        // Step 2: Create depth mask for hand range
        cv::Mat depth_mask = createDepthMask(depth, hand_min_depth_, hand_max_depth_, debug);
        
        // Step 3: Combine masks (skin AND depth)
        cv::Mat combined_mask;
        cv::bitwise_and(skin_mask, depth_mask, combined_mask);
        
        if (debug) {
            int skin_pixels = cv::countNonZero(skin_mask);
            int depth_pixels = cv::countNonZero(depth_mask);
            int combined_pixels = cv::countNonZero(combined_mask);
            std::cout << "[HANDS] Skin pixels: " << skin_pixels 
                      << " Depth pixels: " << depth_pixels 
                      << " Combined: " << combined_pixels << std::endl;
        }
        
        // Step 4: Clean up mask with morphological operations
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_CLOSE, kernel);
        
        // Step 5: Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combined_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (debug) {
            std::cout << "[HANDS] Found " << contours.size() << " contours" << std::endl;
        }
        
        // Step 6: Filter and extract hand positions
        for (size_t i = 0; i < contours.size(); ++i) {
            const auto& contour = contours[i];
            double area = cv::contourArea(contour);
            
            if (debug && area > min_hand_area_ / 2) {  // Log larger contours
                std::cout << "[HANDS] Contour " << i << " area: " << area 
                          << " (range: " << min_hand_area_ << "-" << max_hand_area_ << ")" << std::endl;
            }
            
            if (area >= min_hand_area_ && area <= max_hand_area_) {
                // Calculate centroid
                cv::Moments moments = cv::moments(contour);
                if (moments.m00 > 0) {
                    int cx = static_cast<int>(moments.m10 / moments.m00);
                    int cy = static_cast<int>(moments.m01 / moments.m00);
                    
                    // Get depth at centroid and convert to 3D
                    cv::Point3f world_pos = pixelToWorldCoordinates(cx, cy, depth);
                    if (world_pos.z > 0) {  // Valid depth
                        float confidence = calculateConfidence(area, min_hand_area_, max_hand_area_);
                        hands.emplace_back(world_pos.x, world_pos.y, world_pos.z, "hand", confidence);
                        
                        if (debug) {
                            std::cout << "[HANDS] Hand detected at pixel (" << cx << "," << cy 
                                      << ") world (" << world_pos.x << "," << world_pos.y << "," << world_pos.z 
                                      << ") conf=" << confidence << std::endl;
                        }
                    }
                }
            }
        }
        
        return hands;
    }
    
    std::vector<SimpleDetectedObject> detectTableObjects(const cv::Mat& depth, bool debug = false) {
        std::vector<SimpleDetectedObject> objects;
        
        // Step 1: Create depth mask for table/object range  
        cv::Mat depth_mask = createDepthMask(depth, object_min_depth_, object_max_depth_, debug);
        
        if (debug) {
            int depth_pixels = cv::countNonZero(depth_mask);
            std::cout << "[OBJECTS] Depth pixels in range: " << depth_pixels << std::endl;
        }
        
        // Step 2: Clean up mask (remove noise, fill holes)
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_CLOSE, kernel);
        
        // Step 3: Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(depth_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (debug) {
            std::cout << "[OBJECTS] Found " << contours.size() << " contours" << std::endl;
        }
        
        // Step 4: Filter and extract object positions
        for (size_t i = 0; i < contours.size(); ++i) {
            const auto& contour = contours[i];
            double area = cv::contourArea(contour);
            
            if (debug && area > min_object_area_ / 2) {  // Log larger contours
                std::cout << "[OBJECTS] Contour " << i << " area: " << area 
                          << " (min: " << min_object_area_ << ")" << std::endl;
            }
            
            if (area >= min_object_area_) {
                // Calculate centroid
                cv::Moments moments = cv::moments(contour);
                if (moments.m00 > 0) {
                    int cx = static_cast<int>(moments.m10 / moments.m00);
                    int cy = static_cast<int>(moments.m01 / moments.m00);
                    
                    // Convert to 3D coordinates
                    cv::Point3f world_pos = pixelToWorldCoordinates(cx, cy, depth);
                    if (world_pos.z > 0) {  // Valid depth
                        float confidence = calculateConfidence(area, min_object_area_, min_object_area_ * 5);
                        objects.emplace_back(world_pos.x, world_pos.y, world_pos.z, "object", confidence);
                        
                        if (debug) {
                            std::cout << "[OBJECTS] Object detected at pixel (" << cx << "," << cy 
                                      << ") world (" << world_pos.x << "," << world_pos.y << "," << world_pos.z 
                                      << ") conf=" << confidence << std::endl;
                        }
                    }
                }
            }
        }
        
        return objects;
    }
    
    cv::Mat createSkinMask(const cv::Mat& rgb, bool debug = false) {
        cv::Mat hsv, mask;
        
        // Try RGB first (what we expect from Kinect)
        cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
        cv::inRange(hsv, lower_skin_hsv_, upper_skin_hsv_, mask);
        
        // Additional range for different lighting
        cv::Mat mask2;
        cv::inRange(hsv, cv::Scalar(160, 48, 80), cv::Scalar(180, 255, 255), mask2);
        cv::bitwise_or(mask, mask2, mask);
        
        int skin_pixels_rgb = cv::countNonZero(mask);
        
        // If RGB doesn't work well, try BGR (fallback)
        if (skin_pixels_rgb < 100) {  // Very few skin pixels found
            cv::Mat hsv_bgr, mask_bgr;
            cv::cvtColor(rgb, hsv_bgr, cv::COLOR_BGR2HSV);
            cv::inRange(hsv_bgr, lower_skin_hsv_, upper_skin_hsv_, mask_bgr);
            
            cv::Mat mask2_bgr;
            cv::inRange(hsv_bgr, cv::Scalar(160, 48, 80), cv::Scalar(180, 255, 255), mask2_bgr);
            cv::bitwise_or(mask_bgr, mask2_bgr, mask_bgr);
            
            int skin_pixels_bgr = cv::countNonZero(mask_bgr);
            
            if (debug) {
                std::cout << "[SKIN] RGB mode: " << skin_pixels_rgb << " pixels, BGR mode: " << skin_pixels_bgr << " pixels";
            }
            
            // Use BGR if it gives significantly more skin pixels
            if (skin_pixels_bgr > skin_pixels_rgb * 2) {
                mask = mask_bgr;
                if (debug) {
                    std::cout << " -> Using BGR";
                }
            } else if (debug) {
                std::cout << " -> Using RGB";
            }
            
            if (debug) {
                std::cout << std::endl;
            }
        }
        
        if (debug) {
            cv::Scalar rgb_mean = cv::mean(rgb);
            int final_skin_pixels = cv::countNonZero(mask);
            std::cout << "[SKIN] RGB mean: (" << rgb_mean[0] << "," << rgb_mean[1] << "," << rgb_mean[2] 
                      << ") Final skin pixels: " << final_skin_pixels << std::endl;
        }
        
        return mask;
    }
    
    cv::Mat createDepthMask(const cv::Mat& depth, float min_depth_mm, float max_depth_mm, bool debug = false) {
        cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8UC1);
        
        int valid_pixels = 0;
        int range_pixels = 0;
        
        depth.forEach<uint16_t>([&](uint16_t& pixel, const int position[2]) {
            float depth_mm = static_cast<float>(pixel);
            if (isValidDepth(depth_mm)) {
                valid_pixels++;
                if (depth_mm >= min_depth_mm && depth_mm <= max_depth_mm) {
                    mask.at<uint8_t>(position[0], position[1]) = 255;
                    range_pixels++;
                }
            }
        });
        
        if (debug) {
            cv::Scalar depth_mean = cv::mean(depth);
            std::cout << "[DEPTH] Mean: " << depth_mean[0] << "mm Range: " << min_depth_mm 
                      << "-" << max_depth_mm << "mm Valid: " << valid_pixels 
                      << " InRange: " << range_pixels << std::endl;
        }
        
        return mask;
    }
    
    cv::Point3f pixelToWorldCoordinates(int pixel_x, int pixel_y, const cv::Mat& depth) {
        // Get depth value at pixel
        if (pixel_x < 0 || pixel_x >= depth.cols || pixel_y < 0 || pixel_y >= depth.rows) {
            return cv::Point3f(0, 0, 0);  // Invalid
        }
        
        uint16_t depth_mm = depth.at<uint16_t>(pixel_y, pixel_x);
        if (!isValidDepth(static_cast<float>(depth_mm))) {
            return cv::Point3f(0, 0, 0);  // Invalid
        }
        
        // Convert to meters
        float depth_m = static_cast<float>(depth_mm) / 1000.0f;
        
        // Convert pixel coordinates to world coordinates using camera intrinsics
        float world_x = (static_cast<float>(pixel_x) - cx_) * depth_m / fx_;
        float world_y = (static_cast<float>(pixel_y) - cy_) * depth_m / fy_;
        
        return cv::Point3f(world_x, world_y, depth_m);
    }
    
    bool isValidDepth(float depth_mm) {
        return depth_mm > 0 && depth_mm < 10000;  // Valid Kinect range
    }
    
    float calculateConfidence(double area, double min_area, double max_area) {
        // Simple confidence based on how close area is to expected range
        double normalized_area = (area - min_area) / (max_area - min_area);
        normalized_area = std::max(0.0, std::min(1.0, normalized_area));
        
        // Peak confidence at middle of range
        float confidence = 1.0f - std::abs(static_cast<float>(normalized_area) - 0.5f) * 2.0f;
        return std::max(0.1f, std::min(1.0f, confidence));
    }
    
    // Camera intrinsic parameters (Kinect defaults from existing code)
    float fx_, fy_, cx_, cy_;
    
    // Detection parameters
    int min_hand_area_, max_hand_area_, min_object_area_;
    float hand_min_depth_, hand_max_depth_;
    float object_min_depth_, object_max_depth_;
    
    // Skin color range
    cv::Scalar lower_skin_hsv_, upper_skin_hsv_;
};