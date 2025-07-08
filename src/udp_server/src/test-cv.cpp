#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "capture-cv.hpp"
#include "utils.hpp"

using namespace cv;


int main(int argc, const char** argv)
{
    auto cmap = get_cmap();

    CVKinectCapture capture;

    capture.set_rgb_callback([](cv::Mat& rgb, uint32_t timestamp) {

        imshow("RGB", rgb);
    });

    capture.set_depth_callback([&](cv::Mat& depth, uint32_t timestamp) {
        cv::Mat_<uint16_t> depth16 = depth;

        cv::Mat depth_rgb(depth.rows, depth.cols, CV_8UC3);
        depth16.forEach([&](uint16_t& pixel, const int position[2]) {
            rgb8 color = cmap[pixel];
            depth_rgb.at<cv::Vec3b>(position[0], position[1]) = cv::Vec3b(color.r, color.g, color.b);
        });
        imshow("Depth", depth_rgb);
    });

    // Create windows
    namedWindow("RGB", WINDOW_AUTOSIZE);
    namedWindow("Depth", WINDOW_AUTOSIZE);

    // Add a button for calibration
    //createButton("Calibrate", [](int state, void* userdata) {
    //    std::cout << "Calibrating..." << std::endl;
    //}, NULL, QT_PUSH_BUTTON, 0);


    capture.start();
    while (true) {
        capture.next_loop_event();
        if (waitKey(1) >= 0)
            break;
    }
    capture.stop();

    // Destroy all windows
    destroyAllWindows();

    return 0;
}