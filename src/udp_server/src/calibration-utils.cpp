#include "calibration-utils.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

cv::Mat unwrap_estimate(std::vector<cv::Point2f> input_points, int width, int height, bool mirror)
{
    std::vector<cv::Point2f> output_points = {
        cv::Point2f(0, 0),
        cv::Point2f(width, 0),
        cv::Point2f(width, height),
        cv::Point2f(0, height)
    };

    if (mirror) {
        std::swap(input_points[0], input_points[2]);
        std::swap(input_points[1], input_points[3]);
    }

    return cv::findHomography(input_points, output_points);
}

// Return a new image unwraped from the wrapped image
cv::Mat unwrap(const cv::Mat& wrapped, const cv::Mat& H)
{
    cv::Mat im_out;
    // Warp source image to destination based on homography
    cv::warpPerspective(wrapped, im_out, H, wrapped.size());
    return im_out;
}

QImage get_calibration_image(int width, int height)
{
    QImage img(width, height, QImage::Format_RGB32);
    img.fill(qRgb(0, 0, 0));


    // Put a cross on the center of the image
    for (int h = height / 2; h > 0; h -= 50)
    {
        for (int x = 0; x < width; ++x) {
            img.setPixel(x, h, qRgb(255, 0, 0));
            img.setPixel(x, height - h - 1, qRgb(255, 0, 0));
        }
    }

    for (int w = width / 2; w > 0; w -= 50)
    {
        for (int y = 0; y < height; ++y) {
            img.setPixel(w, y, qRgb(0, 255, 0));
            img.setPixel(width - w - 1, y, qRgb(0, 255, 0));
        }
    }

    // Top left corner
    for (int x = 0; x < 50; ++x)
        for (int y = 0; y < 50; ++y)
            img.setPixel(x, y, qRgb(0, 0, 255));


    return img;
}