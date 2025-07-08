#include "calibrate-qt.hpp"
#include "utils.hpp"

cv::Mat depthmap_colorize(cv::Mat _depth, int min_depth, int max_depth)
{
    static auto cmap = get_cmap(5.f);
    cv::Mat_<uint16_t> depth16 = _depth;

    // Scale the depth image
    if (min_depth > 0 && max_depth > 0)
    {
        min_depth = std::clamp((int)(0.75f * min_depth), 0, 2047);
        max_depth = std::clamp((int)(1.25f * max_depth), 0, 2047);
        depth16.forEach([&](uint16_t &pixel, const int position[2]) {
            int value = (pixel - min_depth) * 2048 / (max_depth - min_depth);
            pixel = std::clamp(value, 0, 2047);
        });
    }


    // Colorize the unwrapped depth image
    cv::Mat depth_rgb(depth16.rows, depth16.cols, CV_8UC3);
    depth16.forEach([&](uint16_t &pixel, const int position[2]) {
        rgb8 color = cmap[pixel];
        depth_rgb.at<cv::Vec3b>(position[0], position[1]) = cv::Vec3b(color.r, color.g, color.b); }
    );

    return depth_rgb;
}

int main(int argc, char** argv)
{
    // Create a QT application with a window and side-by-side RGB and Depth panel
    QApplication app(argc, argv);

    QCalibrationApp win;
    win.setOnDepthFrameChange(depthmap_colorize);
    win.show();

    return app.exec();
}