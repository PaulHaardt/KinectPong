#include <iostream>
#include <opencv2/opencv.hpp>

#include <OpenNI.h>

int main() {
  // Initialize OpenNI
  openni::Status rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    std::cerr << "Initialize failed: " << openni::OpenNI::getExtendedError()
              << std::endl;
    return -1;
  }

  // Open device
  openni::Device device;
  rc = device.open(openni::ANY_DEVICE);
  if (rc != openni::STATUS_OK) {
    std::cerr << "Couldn't open device: " << openni::OpenNI::getExtendedError()
              << std::endl;
    return -1;
  }

  // Create depth stream
  openni::VideoStream depth;
  rc = depth.create(device, openni::SENSOR_DEPTH);
  if (rc != openni::STATUS_OK) {
    std::cerr << "Couldn't create depth stream: "
              << openni::OpenNI::getExtendedError() << std::endl;
    return -1;
  }

  rc = depth.start();
  if (rc != openni::STATUS_OK) {
    std::cerr << "Couldn't start depth stream: "
              << openni::OpenNI::getExtendedError() << std::endl;
    return -1;
  }

  cv::Mat imgGray, depth8, depthEq, mask, filtered_mask;
  double minVal, maxVal;

  while (true) {
    // Read frame from Kinect
    openni::VideoFrameRef frame;
    rc = depth.readFrame(&frame);
    if (rc != openni::STATUS_OK) {
      continue;
    }

    // Convert to OpenCV Mat
    cv::Mat rawDepth(frame.getHeight(), frame.getWidth(), CV_16UC1,
                     (void *)frame.getData());

    // Create a copy to avoid issues with frame buffer
    cv::Mat depthMat = rawDepth.clone();

    // Convert depth to grayscale (depth is already single channel)
    // For depth data, we don't need cvtColor - it's already grayscale
    depthMat.convertTo(imgGray, CV_8UC1, 255.0 / 65535.0); // Normalize 16-bit to 8-bit

    // Find min/max values on the actual depth data
    cv::minMaxLoc(depthMat, &minVal, &maxVal);

    // Avoid divide-by-zero
    if (maxVal - minVal < 1e-6) {
      std::cerr << "Invalid depth range: " << minVal << " - " << maxVal
                << std::endl;
      continue;
    }

    // Better normalization for depth data
    depthMat.convertTo(depth8, CV_8UC1, 255.0 / (maxVal - minVal),
                      -minVal * 255.0 / (maxVal - minVal));

    // Histogram equalization
    cv::equalizeHist(depth8, depthEq);

    // Threshold to isolate hand and objects
    cv::inRange(depthEq, 0, 20, mask);

    // Morphological operations
    cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // Filter objects that don't touch the border
    cv::Mat labels, stats, centroids;
    int num_labels =
        cv::connectedComponentsWithStats(mask, labels, stats, centroids);

    filtered_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
    int rows = mask.rows;
    int cols = mask.cols;

    // Check each connected component
    for (int i = 1; i < num_labels; i++) {
      int left = stats.at<int>(i, cv::CC_STAT_LEFT);
      int top = stats.at<int>(i, cv::CC_STAT_TOP);
      int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
      int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
      int right = left + width - 1;
      int bottom = top + height - 1;

      bool touches_border = false;

      if (left == 0 || right == cols - 1 || top == 0 || bottom == rows - 1) {
        touches_border = true;
      } else {
        cv::Mat component_mask = (labels == i);

        for (int x = left; x <= right && !touches_border; x++) {
          if (component_mask.at<int>(top, x) != 0 ||
              component_mask.at<int>(bottom, x) != 0) {
            touches_border = true;
          }
        }

        for (int y = top; y <= bottom && !touches_border; y++) {
          if (component_mask.at<int>(y, left) != 0 ||
              component_mask.at<int>(y, right) != 0) {
            touches_border = true;
          }
        }
      }

      if (touches_border) {
        filtered_mask.setTo(255, labels == i);
      }
    }

    // Find farthest point from any border for each border-touching object
    cv::Mat visualization;
    cv::cvtColor(filtered_mask, visualization, cv::COLOR_GRAY2BGR);

    // Clear console output for real-time updates (optional)
    // std::cout << "\033[2J\033[1;1H"; // Clear screen (Unix/Linux)

    for (int i = 1; i < num_labels; i++) {
      cv::Mat component_mask = (labels == i);
      bool component_kept = false;

      cv::Mat component_in_filtered;
      cv::bitwise_and(component_mask, filtered_mask, component_in_filtered);
      if (cv::countNonZero(component_in_filtered) > 0) {
        component_kept = true;
      }

      if (component_kept) {
        cv::Point farthest_point(-1, -1);
        int max_distance = -1;

        for (int y = 0; y < labels.rows; y++) {
          for (int x = 0; x < labels.cols; x++) {
            if (labels.at<int>(y, x) == i) {
              int dist_to_left = x;
              int dist_to_right = (labels.cols - 1) - x;
              int dist_to_top = y;
              int dist_to_bottom = (labels.rows - 1) - y;

              int min_border_distance = std::min(
                  {dist_to_left, dist_to_right, dist_to_top, dist_to_bottom});

              if (min_border_distance > max_distance) {
                max_distance = min_border_distance;
                farthest_point = cv::Point(x, y);
              }
            }
          }
        }

        if (farthest_point.x != -1) {
          cv::Scalar color = cv::Scalar(0, 255, 0);

          cv::circle(visualization, farthest_point, 8, color, -1);

          // Print coordinates (optional - can be commented out to reduce
          // console spam)
          std::cout << "Frame - Component " << i << ": (" << farthest_point.x
                    << ", " << farthest_point.y << ") Dist: " << max_distance
                    << std::endl;
        }
      }
    }

    // Display results - now using cv::Mat objects
    cv::imshow("Original Frame", depth8);  // Show the processed depth as grayscale
    cv::imshow("Processed Mask", filtered_mask);
    cv::imshow("Farthest Points", visualization);

    // Exit condition
    char key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) {
      break;
    }
  }

  depth.stop();
  depth.destroy();
  device.close();
  openni::OpenNI::shutdown();
  return 0;
}