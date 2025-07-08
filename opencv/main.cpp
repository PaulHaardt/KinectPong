
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
  // Open video capture (0 for default camera, or specify video file path)
  cv::VideoCapture cap("recordings/depth_capture.avi");
  // For video file, use: cv::VideoCapture cap("path/to/video.mp4");

  if (!cap.isOpened()) {
    std::cerr << "Error: Cannot open video source" << std::endl;
    return -1;
  }

  cv::Mat frame, imgGray, depth8, depthEq, mask, filtered_mask;
  double minVal, maxVal;

  while (true) {
    // Capture frame
    cap >> frame;
    if (frame.empty()) {
      std::cerr << "End of video or failed to capture frame" << std::endl;
      break;
    }

    // Convert to grayscale (assuming input is RGB, adjust if depth camera)
    cv::cvtColor(frame, imgGray, cv::COLOR_BGR2GRAY);

    // Find min/max values
    cv::minMaxLoc(imgGray, &minVal, &maxVal);

    // Avoid divide-by-zero
    if (maxVal - minVal < 1e-6) {
      std::cerr << "Invalid depth range: " << minVal << " - " << maxVal
                << std::endl;
      continue;
    }

    // Normalize to 8-bit
    imgGray.convertTo(depth8, CV_8UC1, 255.0 / (maxVal - minVal),
                      -minVal * 255.0 / (maxVal - minVal));

    // Histogram equalization
    cv::equalizeHist(depth8, depthEq);

    // Threshold to isolate hand and objects
    cv::inRange(depthEq, 0, 100, mask);

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

    // Display results
    cv::imshow("Original Frame", frame);
    cv::imshow("Processed Mask", filtered_mask);
    cv::imshow("Farthest Points", visualization);

    // Exit on 'q' key press or ESC
    char key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) {
      break;
    }
  }

  cap.release();
  cv::destroyAllWindows();
  return 0;
}
