#pragma once

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

struct SimpleDetectedObject {
    float x, y, z;
    int id;

    SimpleDetectedObject() : x(0.0f), y(0.0f), z(0.0f), id(-1) {}


    SimpleDetectedObject(float x_, float y_, float z_, int id_)
        : x(x_), y(y_), z(z_), id(id_) {}
};

class SimpleDetector {
public:
    SimpleDetector() {
        low_threshold = 0;
        high_threshold = 40;
        input_width = 640.0f;
        input_height = 480.0f;
        homography = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat h2 = cv::Mat::eye(3, 3, CV_64F);

        readHomographies("calibration.yml", homography, h2);
        // prints the homography matrix
        std::cout << "Homography matrix: " << std::endl;
        std::cout << homography << std::endl;
    }

    void setHomography(const cv::Mat &H) {
        if (H.size() == cv::Size(3, 3)) {
            H.copyTo(homography);
        } else {
            std::cerr << "Invalid homography matrix size!" << std::endl;
        }
    }

    std::pair<std::vector<SimpleDetectedObject>,
              std::vector<SimpleDetectedObject>>
    detectObjects(const cv::Mat &rgb, const cv::Mat &depth) {
        std::vector<SimpleDetectedObject> hands;
        std::vector<SimpleDetectedObject> objects;

        if (rgb.empty() || depth.empty()) {
            return {hands, objects};
        }

        cv::Mat imgGray = depth.clone();
        cv::Mat depth8, depthEq, mask;
        double minVal, maxVal;
        cv::minMaxLoc(imgGray, &minVal, &maxVal);

        if (maxVal - minVal < 1e-6) {
            std::cerr << "Invalid depth range: " << minVal << " - " << maxVal
                      << std::endl;
            return {hands, objects};
        }

        imgGray.convertTo(depth8, CV_8UC1, 255.0 / (maxVal - minVal),
                          -minVal * 255.0 / (maxVal - minVal));
        cv::equalizeHist(depth8, depthEq);
        cv::inRange(depthEq, low_threshold, high_threshold, mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        cv::Mat labels, stats, centroids;
        int num_labels =
                cv::connectedComponentsWithStats(mask, labels, stats, centroids);
        cv::Mat filtered_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
        int rows = mask.rows;
        int cols = mask.cols;

        std::vector<int> id_list;

        for (int i = 1; i < num_labels; i++) {
            int left = stats.at<int>(i, cv::CC_STAT_LEFT);
            int top = stats.at<int>(i, cv::CC_STAT_TOP);
            int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            int right = left + width - 1;
            int bottom = top + height - 1;

            id_list.push_back(-1);
            bool touches_border = false;
            cv::Mat component_mask = (labels == i);

            for (int y = top; y <= bottom && !touches_border; y++) {
                if (component_mask.at<int>(y, left) != 0) {
                    id_list[i - 1] = 0;
                    touches_border = true;
                }
                if (component_mask.at<int>(y, right) != 0) {
                    id_list[i - 1] = 1;
                    touches_border = true;
                }
            }

            if (touches_border) {
                filtered_mask.setTo(255, labels == i);
            }
        }

        // Find farthest point from any border for each border-touching object
        cv::Mat visualization;
        cv::cvtColor(filtered_mask, visualization, cv::COLOR_GRAY2BGR);

        for (int i = 1; i < num_labels; i++) {
            cv::Mat component_mask = (labels == i);
            cv::Mat component_in_filtered;
            cv::bitwise_and(component_mask, filtered_mask, component_in_filtered);
            if (cv::countNonZero(component_in_filtered) == 0)
                continue;

            cv::Point farthest_point(-1, -1);
            int max_distance = -1;

            for (int y = 0; y < labels.rows; y++) {
                for (int x = 0; x < labels.cols; x++) {
                    if (labels.at<int>(y, x) == i) {
                        int dist_to_left = x;
                        int dist_to_right = (labels.cols - 1) - x;
                        int min_border_distance = std::min({dist_to_left, dist_to_right});
                        if (min_border_distance > max_distance) {
                            max_distance = min_border_distance;
                            farthest_point = cv::Point(x, y);
                        }
                    }
                }
            }

            if (farthest_point.x != -1) {
                cv::Scalar color = cv::Scalar(0, 255, 0);

                // Apply homography to farthest_point
                std::vector<cv::Point2f> input_pts = {cv::Point2f(farthest_point.x, farthest_point.y)};
                std::vector<cv::Point2f> output_pts;
                cv::perspectiveTransform(input_pts, output_pts, homography);

                cv::Point2f transformed_point = output_pts[0];
                cv::circle(visualization, output_pts[0], 8, color, -1);

                cv::Mat pt = (cv::Mat_<double>(3, 1) << output_pts[0].x / input_width,
                              1 - output_pts[0].y / input_height, 1.0);
                float x = pt.at<double>(0, 0) / pt.at<double>(2, 0);
                float y = pt.at<double>(1, 0) / pt.at<double>(2, 0);
                float z = 0;// You could get depth value here if needed
                if (x < 0.03 || x > 0.97 || y < 0.03 || y > 0.97) {
                    continue;// Skip points too close to the border
                }
                int id = id_list[i - 1];

                hands.push_back(SimpleDetectedObject(x, y, z, id));
            }
        }

        cv::imshow("RGB", rgb);
        cv::imshow("Processed", visualization);
        cv::imshow("Depth", depthEq);
        cv::waitKey(1);

        return {hands, objects};
    }

private:
    int low_threshold;
    int high_threshold;
    float input_width;
    float input_height;

    cv::Mat homography;
};
