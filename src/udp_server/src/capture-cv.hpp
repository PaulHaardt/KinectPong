#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <functional>

class CVKinectCapture
{
    public:
        enum resolution {
            LOW = 0,
            MEDIUM = 1,
            HIGH = 2
        };


        CVKinectCapture(resolution video_res = MEDIUM, resolution depth_res = MEDIUM);
        ~CVKinectCapture();

        void set_rgb_callback(std::function<void(cv::Mat&, uint32_t)> cb);
        void set_depth_callback(std::function<void(cv::Mat&, uint32_t)> cb);

        // Must be called from the main thread
        void start();
        void next_loop_event();
        void stop();

    private:
        struct FreenectContext;

        static void depth_cb_wrapper(void* dev, void* data, uint32_t timestamp);
        static void rgb_cb_wrapper(void* dev, void* data, uint32_t timestamp);
        static std::function<void(cv::Mat&, uint32_t timestamp)> rgb_cb;
        static std::function<void(cv::Mat&, uint32_t timestamp)> depth_cb;

        std::unique_ptr<FreenectContext> ctx; 
        bool running = false;
};


