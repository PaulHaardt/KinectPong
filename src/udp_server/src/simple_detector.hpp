#pragma once

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

// Reuse the same structures from our main server
struct SimpleDetectedObject {
  float x, y, z;
  int id;

  SimpleDetectedObject(float x_, float y_, float z_, int id_)
      : x(x_), y(y_), z(z_), id(id_) {}
};

class SimpleDetector {
public:
  SimpleDetector() {
    low_threshold = 0;
    high_threshold = 10;
  }

  // Main detection function
  std::pair<std::vector<SimpleDetectedObject>,
            std::vector<SimpleDetectedObject>>
  detectObjects(const cv::Mat &rgb, const cv::Mat &depth) {

    std::vector<SimpleDetectedObject> hands;
    std::vector<SimpleDetectedObject> objects;

    // Handle empty frames gracefully (dummy mode or initialization)
    if (rgb.empty() || depth.empty()) {
      return {hands, objects};
    }

    cv::Mat imgGray; // Result grayscale image
    cv::cvtColor(depth, imgGray, cv::COLOR_RGBA2GRAY);

    cv::Mat depth8, depthEq, mask;
    double minVal, maxVal;
    cv::minMaxLoc(imgGray, &minVal, &maxVal);

    // Avoid divide-by-zero
    if (maxVal - minVal < 1e-6) {
      std::cerr << "Invalid depth range: " << minVal << " - " << maxVal
                << std::endl;
      return -1;
    }

    // Normalize to 8-bit
    imgGray.convertTo(depth8, CV_8UC1, 255.0 / (maxVal - minVal),
                      -minVal * 255.0 / (maxVal - minVal));

    // Histogram equalization
    cv::equalizeHist(depth8, depthEq);

    // Threshold to isolate hand and objects (expanded range)
    cv::inRange(depthEq, low_threshold, high_threshold, mask);

    // Improved morphological operations
    cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);  // Remove noise
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel); // Fill holes

    // Filter objects that don't touch the border
    cv::Mat labels, stats, centroids;
    int num_labels =
        cv::connectedComponentsWithStats(mask, labels, stats, centroids);

    cv::Mat filtered_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
    int rows = mask.rows;
    int cols = mask.cols;

    // Check each connected component
    for (int i = 1; i < num_labels; i++) {
      // Get bounding box of the component
      int left = stats.at<int>(i, cv::CC_STAT_LEFT);
      int top = stats.at<int>(i, cv::CC_STAT_TOP);
      int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
      int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
      int right = left + width - 1;
      int bottom = top + height - 1;

      // Check if component touches any border
      bool touches_border = false;

      // Check if bounding box touches border
      if (left == 0 || right == cols - 1 || top == 0 || bottom == rows - 1) {
        touches_border = true;
      } else {
        // More precise check: scan actual pixels at border positions
        cv::Mat component_mask = (labels == i);

        // Check top and bottom rows
        for (int x = left; x <= right && !touches_border; x++) {
          if (component_mask.at<int>(top, x) != 0 ||
              component_mask.at<int>(bottom, x) != 0) {
            touches_border = true;
          }
        }

        // Check left and right columns
        for (int y = top; y <= bottom && !touches_border; y++) {
          if (component_mask.at<int>(y, left) != 0 ||
              component_mask.at<int>(y, right) != 0) {
            touches_border = true;
          }
        }
      }

      // Keep only components that touch the border
      if (touches_border) {
        filtered_mask.setTo(255, labels == i);
      }
    }

    // Find farthest point from any border for each border-touching object
    cv::Mat visualization;
    cv::cvtColor(filtered_mask, visualization, cv::COLOR_GRAY2BGR);

    // For each component that touches the border, find its farthest point
    for (int i = 1; i < num_labels; i++) {
      // Check if this component was kept (touches border)
      cv::Mat component_mask = (labels == i);
      bool component_kept = false;

      // Quick check if component exists in filtered mask
      cv::Mat component_in_filtered;
      cv::bitwise_and(component_mask, filtered_mask, component_in_filtered);
      if (cv::countNonZero(component_in_filtered) > 0) {
        component_kept = true;
      }

      if (component_kept) {
        cv::Point farthest_point(-1, -1);
        int max_distance = -1;

        // Find all pixels of this component
        for (int y = 0; y < labels.rows; y++) {
          for (int x = 0; x < labels.cols; x++) {
            if (labels.at<int>(y, x) == i) {
              // Calculate minimum distance to any border
              int dist_to_left = x;
              int dist_to_right = (labels.cols - 1) - x;
              int dist_to_top = y;
              int dist_to_bottom = (labels.rows - 1) - y;

              int min_border_distance = std::min(
                  {dist_to_left, dist_to_right, dist_to_top, dist_to_bottom});

              // Keep track of the point farthest from any border
              if (min_border_distance > max_distance) {
                max_distance = min_border_distance;
                farthest_point = cv::Point(x, y);
              }
            }
          }
        }

        // Draw dot and print coordinates for this component
        if (farthest_point.x != -1) {
          SimpleDetectedObject object =
              SimpleDetectedObject(farthest_point.x, farthest_point.y, 0, i);
          objects.push_back(object);
        }
      }
    }

    return {hands, objects};
  }

private:
  int low_threshold;
  int high_threshold;
};
