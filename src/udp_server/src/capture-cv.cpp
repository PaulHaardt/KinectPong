#include "capture-cv.hpp"

#include <opencv2/imgproc.hpp>
#include <libfreenect/libfreenect.h>

struct CVKinectCapture::FreenectContext
{
    freenect_context* fn_ctx = nullptr;
    freenect_device* fn_dev = nullptr;

    FreenectContext()
    {
        int ret = freenect_init(&fn_ctx, NULL);
        if (ret < 0) {
            throw std::runtime_error("Failed to initialize freenect");
        }

        freenect_set_log_level(fn_ctx, FREENECT_LOG_DEBUG);
        freenect_select_subdevices(fn_ctx, FREENECT_DEVICE_CAMERA);
    }

    ~FreenectContext()
    {
        if (fn_dev != nullptr)
            freenect_close_device(fn_dev);
        if (fn_ctx != nullptr)
            freenect_shutdown(fn_ctx);
    }
};



CVKinectCapture::CVKinectCapture(resolution video_res, resolution depth_res)
{
    ctx = std::make_unique<FreenectContext>();
    auto fn_ctx = ctx->fn_ctx;

    int num_devices = freenect_num_devices(fn_ctx);
    if (num_devices < 0) {
        throw std::runtime_error("Failed to get number of devices");
    }

    if (num_devices == 0) {
        throw std::runtime_error("No device found!");
    }

    if (freenect_open_device(fn_ctx, &ctx->fn_dev, 0) < 0)
        throw std::runtime_error("Failed to open device");

    auto mode_depth = freenect_find_depth_mode((freenect_resolution) depth_res, FREENECT_DEPTH_11BIT);
    if (freenect_set_depth_mode(ctx->fn_dev, mode_depth) < 0)
        throw std::runtime_error("Failed to set depth mode");


    auto mode_rgb = freenect_find_video_mode((freenect_resolution) video_res, FREENECT_VIDEO_RGB);
    if (freenect_set_video_mode(ctx->fn_dev, mode_rgb) < 0)
        throw std::runtime_error("Failed to set video mode");

    
    freenect_set_video_callback(ctx->fn_dev, (freenect_video_cb) rgb_cb_wrapper);
    freenect_set_depth_callback(ctx->fn_dev, (freenect_depth_cb) depth_cb_wrapper);
}

CVKinectCapture::~CVKinectCapture()
{
    if (running)
        stop();
}

void CVKinectCapture::set_rgb_callback(std::function<void(cv::Mat&, uint32_t)> cb)
{
    rgb_cb = std::move(cb);
}

void CVKinectCapture::set_depth_callback(std::function<void(cv::Mat&, uint32_t)> cb)
{
    depth_cb = std::move(cb);
}

void CVKinectCapture::depth_cb_wrapper(void* _dev, void* data, uint32_t timestamp)
{
    auto dev = static_cast<freenect_device*>(_dev);
    auto mode = freenect_get_current_depth_mode(dev);
    auto depthmap = cv::Mat(mode.height, mode.width, CV_16UC1, data);
    if (depth_cb) {
        depth_cb(depthmap, timestamp);
    }
}

void CVKinectCapture::rgb_cb_wrapper(void* _dev, void* data, uint32_t timestamp)
{
    auto dev = static_cast<freenect_device*>(_dev);
    auto mode = freenect_get_current_video_mode(dev);
    auto rgbmap = cv::Mat(mode.height, mode.width, CV_8UC3, data);
    if (rgb_cb) {
        // Convert RGB to BGR
        //cv::cvtColor(rgbmap, rgbmap, cv::COLOR_RGB2BGR);
        rgb_cb(rgbmap, timestamp);
    }
}


void CVKinectCapture::start()
{
    if (freenect_start_depth(ctx->fn_dev) < 0)
        throw std::runtime_error("Failed to start depth stream");

    if (freenect_start_video(ctx->fn_dev) < 0)
        throw std::runtime_error("Failed to start video stream");

    running = true;
}

void CVKinectCapture::next_loop_event()
{
    //struct timeval timeout = {0, 0};
    //freenect_process_events_timeout(ctx->fn_ctx, &timeout);

    if (freenect_process_events(ctx->fn_ctx) < 0)
        throw std::runtime_error("Failed to process events");
}

void CVKinectCapture::stop()
{
    if (freenect_stop_depth(ctx->fn_dev) < 0)
        throw std::runtime_error("Failed to stop depth stream");

    if (freenect_stop_video(ctx->fn_dev) < 0)
        throw std::runtime_error("Failed to stop video stream");

    running = false;
}


std::function<void(cv::Mat&, uint32_t)> CVKinectCapture::rgb_cb;
std::function<void(cv::Mat&, uint32_t)> CVKinectCapture::depth_cb;