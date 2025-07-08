#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>


#include <libfreenect/libfreenect.h>

unsigned long long IID_RGB = 0u;
unsigned long long IID_DPT = 0u;

volatile bool running = true;
void sighand(int signal)
{
    if (signal == SIGINT || signal == SIGTERM || signal == SIGQUIT)
    {
        running = false;
    }
}

void depth_cb(freenect_device* dev, void* data, uint32_t timestamp)
{
    std::stringstream ss;
    ss << "outs/dpt_" << std::setfill('0') << std::setw(5) << IID_DPT++ << ".ppm";
    std::ofstream outfile(ss.str());
    std::cout << ss.str() << std::endl;

    outfile << "P3\n"
            << "640"
            << " "
            << "480" << '\n'
            << "255" << '\n';

    uint16_t* depth = (uint16_t*)data;
    for (unsigned i = 0u; i < 640u * 480u; ++i)
    {
        int nb = static_cast<int>((static_cast<double>(depth[i]) * 255.l) / 65535.l);
        outfile << nb << ' ' << nb << ' '
                << nb << ' ' << '\n';
    }
}

void rgb_cb(freenect_device* dev, void* data, uint32_t timestamp)
{
    std::stringstream ss;
    ss << "outs/rgb_" << std::setfill('0') << std::setw(5) << IID_RGB++ << ".ppm";
    std::ofstream outfile(ss.str());
    std::cout << ss.str() << std::endl;

    outfile << "P3\n"
            << "640"
            << " "
            << "480" << '\n'
            << "255" << '\n';

    uint8_t* rgb = (uint8_t*)data;
    for (unsigned i = 0u; i < 3u * 640u * 480u; i += 3)
    {
        outfile << (int)rgb[i + 0u] << ' ' << (int)rgb[i + 1u] << ' '
                << (int)rgb[i + 2u] << ' ' << '\n';
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT, sighand);
    signal(SIGTERM, sighand);
    signal(SIGQUIT, sighand);

    freenect_context* fn_ctx;
    int ret = freenect_init(&fn_ctx, NULL);
    if (ret < 0)
        return ret;

    freenect_set_log_level(fn_ctx, FREENECT_LOG_DEBUG);
    freenect_select_subdevices(fn_ctx, FREENECT_DEVICE_CAMERA);

    int num_devices = ret = freenect_num_devices(fn_ctx);
    if (ret < 0)
        return ret;

    if (num_devices == 0)
    {
        std::clog << "No device found!" << std::endl;
        freenect_shutdown(fn_ctx);
        return 1;
    }

    freenect_device* fn_dev;
    ret = freenect_open_device(fn_ctx, &fn_dev, 0);
    if (ret < 0)
    {
        freenect_shutdown(fn_ctx);
        return ret;
    }

    ret = freenect_set_depth_mode(
        fn_dev,
        freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM,
                                 FREENECT_DEPTH_MM));
    if (ret < 0)
    {
        freenect_shutdown(fn_ctx);
        return ret;
    }

    ret = freenect_set_video_mode(
        fn_dev,
        freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM,
                                 FREENECT_VIDEO_RGB));
    if (ret < 0)
    {
        freenect_shutdown(fn_ctx);
        return ret;
    }

    freenect_set_video_callback(fn_dev, rgb_cb);
    freenect_set_depth_callback(fn_dev, depth_cb);

    ret = freenect_start_depth(fn_dev);
    if (ret < 0)
    {
        freenect_shutdown(fn_ctx);
        return ret;
    }
    ret = freenect_start_video(fn_dev);
    if (ret < 0)
    {
        freenect_shutdown(fn_ctx);
        return ret;
    }

    while (running && freenect_process_events(fn_ctx) >= 0)
    {
    }

    std::clog << "Shutting down" << std::endl;

    freenect_stop_depth(fn_dev);
    freenect_stop_video(fn_dev);
    freenect_close_device(fn_dev);
    freenect_shutdown(fn_ctx);

    std::clog << "Done!" << std::endl;

    return 0;
}
