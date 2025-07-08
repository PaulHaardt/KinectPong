#pragma once


#include <cstdint>
#include <opencv2/core.hpp>
#include <QtGui/QImage>



/// @brief \brief Estimate the homography parameters from the coordinates of the corners of the wrapped image
/// @param coordinates [top left, top right, bottom right, bottom left]
/// @param width 
/// @param height 
/// @return  
cv::Mat unwrap_estimate(std::vector<cv::Point2f> coordinates, int width, int height, bool mirror = false);


/// \brief Return a new image unwraped from the wrapped image
/// \param wrapped The image to unwrap
/// \param H The homography matrix 
cv::Mat unwrap(const cv::Mat& wrapped, const cv::Mat& H);


QImage get_calibration_image(int width, int height);