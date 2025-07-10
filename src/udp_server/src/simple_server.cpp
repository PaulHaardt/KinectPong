#include "utils.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <sstream>
#include <atomic>
#include <csignal>
#include <mutex>
#include <iomanip>

// Direct freenect integration (like glview.c)
#include <libfreenect/libfreenect.h>
#include <opencv2/core.hpp>

// Detection
#include "simple_detector.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#endif

#define IP "UDP_IP_UBUNTU"

struct SimpleDetectionResult
{
    std::vector<SimpleDetectedObject> hands;
    std::vector<SimpleDetectedObject> objects;
    uint32_t timestamp;

    SimpleDetectionResult(uint32_t ts) : timestamp(ts) {}
};

class SimpleUDPServer
{
public:
    SimpleUDPServer() : socket_fd_(-1), kinect_mode_(false), detector_() {
#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

        auto env = load_env(".env");
        port_ = std::stoi(env["UDP_SERVER_PORT"]);
        ip_ = env[IP];
        std::cout << "Port: " << port_ << "IP: " << ip_ << std::endl;
    }

    ~SimpleUDPServer()
    {
        stop();
#ifdef _WIN32
        WSACleanup();
#endif
    }

    bool start()
    {
        std::cout << "Starting UDP server..." << std::endl;

        // Start UDP server first
        if (!startUDPServer())
        {
            return false;
        }

        // Try to start Kinect with glview.c architecture
        if (initKinectGlviewStyle())
        {
            kinect_mode_ = true;
            std::cout << "🟢 KINECT MODE: glview.c architecture" << std::endl;
            startKinectThread();
        }
        else
        {
            kinect_mode_ = false;
            std::cout << "🟡 DUMMY MODE: Simulated sensor data (no Kinect detected)" << std::endl;
            setupDummyMode();
        }

        running_ = true;
        std::cout << "System ready!" << std::endl;
        return true;
    }

    void stop()
    {
        running_ = false;
        die_ = true;

        // Stop Kinect thread
        if (kinect_thread_.joinable())
        {
            kinect_thread_.join();
        }

        // Cleanup Kinect (like glview.c)
        if (kinect_mode_)
        {
            cleanupKinect();
        }

        if (socket_fd_ >= 0)
        {
#ifdef _WIN32
            closesocket(socket_fd_);
#else
            close(socket_fd_);
#endif
            socket_fd_ = -1;
        }
    }

    void runServerLoop()
    {
        char buffer[1024];
        sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000; // 50ms
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char *>(&timeout), sizeof(timeout));

        std::cout << "Server listening... Send 'SUBSCRIBE' to receive data" << std::endl;

        auto last_dummy_time = std::chrono::steady_clock::now();

        while (running_)
        {
            // Handle dummy mode timing
            if (!kinect_mode_)
            {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_dummy_time);

                if (elapsed.count() >= 33)
                { // ~30fps
                    generateDummyFrames();
                    last_dummy_time = now;
                }
            }

            // Check for client subscriptions
            ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                              reinterpret_cast<sockaddr *>(&client_addr), &addr_len);

            if (bytes_received > 0)
            {
                buffer[bytes_received] = '\0';
                std::string message(buffer);

                if (message == "SUBSCRIBE")
                {
                    client_addr_ = client_addr;
                    has_client_ = true;

                    std::cout << "Client subscribed: " << inet_ntoa(client_addr.sin_addr)
                              << ":" << ntohs(client_addr.sin_port) << std::endl;

                    std::string ack_msg = kinect_mode_ ? "{\"status\":\"subscribed\", \"mode\":\"kinect\"}" : "{\"status\":\"subscribed\", \"mode\":\"dummy\"}";

                    sendto(socket_fd_, ack_msg.c_str(), ack_msg.length(), 0,
                           reinterpret_cast<const sockaddr *>(&client_addr), sizeof(client_addr));
                }
            }

            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // ========== FREENECT CALLBACKS (glview.c style) - PUBLIC ==========

    static void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
    {
        if (!instance_)
            return;

        uint16_t *depth = (uint16_t *)v_depth;

        std::lock_guard<std::mutex> lock(instance_->gl_backbuf_mutex_);

        // Simple copy for now (glview.c does color mapping here)
        memcpy(instance_->depth_mid_, depth, 640 * 480 * sizeof(uint16_t));
        instance_->got_depth_ = true;

        // Process frame pair if both available
        instance_->processFramePair();
    }

    static void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
    {
        if (!instance_)
            return;

        std::lock_guard<std::mutex> lock(instance_->gl_backbuf_mutex_);

        // Buffer swapping exactly like glview.c
        uint8_t *temp = instance_->rgb_back_;
        instance_->rgb_back_ = instance_->rgb_mid_;
        freenect_set_video_buffer(dev, instance_->rgb_back_); // ← CRITICAL: Reset buffer
        instance_->rgb_mid_ = (uint8_t *)rgb;

        instance_->got_rgb_ = true;

        // Process frame pair if both available
        instance_->processFramePair();
    }

    void processFramePair()
    {
        if (!got_rgb_ || !got_depth_)
            return;

        // Generate our own timestamp
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - kinect_start_time_);
        uint32_t our_timestamp = static_cast<uint32_t>(elapsed.count());

        // Create OpenCV matrices from raw buffers
        cv::Mat rgb_mat(480, 640, CV_8UC3, rgb_mid_);
        cv::Mat depth_mat(480, 640, CV_16UC1, depth_mid_);

        // Clone to avoid buffer issues
        cv::Mat rgb_clone = rgb_mat.clone();
        cv::Mat depth_clone = depth_mat.clone();

        // Process with detection
        SimpleDetectionResult result = processFrames(rgb_clone, depth_clone, our_timestamp);

        // Broadcast to client
        if (has_client_)
        {
            broadcastDetectionResult(result);
        }

        // Reset flags
        got_rgb_ = false;
        got_depth_ = false;

        // Debug
        static int frame_count = 0;
        frame_count++;
        if (frame_count % 30 == 0)
        {
            std::cout << "[GLVIEW] Frame pair #" << frame_count
                      << " - " << result.hands.size() << " hands, "
                      << result.objects.size() << " objects" << std::endl;
        }
    }

    // Static instance for callbacks (PUBLIC)
    static SimpleUDPServer *instance_;

private:
    bool startUDPServer()
    {
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0)
        {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }

        int opt = 1;
        setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<const char *>(&opt), sizeof(opt));

        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
        server_addr_.sin_port = htons(port_);

        if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&server_addr_), sizeof(server_addr_)) < 0)
        {
            std::cerr << "Failed to bind to port " << port_ << std::endl;
            return false;
        }

        std::cout << "UDP Server started on port " << port_ << std::endl;
        return true;
    }

    // ========== KINECT INTEGRATION (glview.c style) ==========

    bool initKinectGlviewStyle()
    {
        try
        {
            std::cout << "Initializing Kinect with glview.c architecture..." << std::endl;

            // Initialize freenect context (like glview.c main())
            if (freenect_init(&f_ctx_, NULL) < 0)
            {
                std::cout << "freenect_init() failed" << std::endl;
                return false;
            }

            freenect_set_log_level(f_ctx_, FREENECT_LOG_DEBUG);
            freenect_select_subdevices(f_ctx_, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

            int nr_devices = freenect_num_devices(f_ctx_);
            std::cout << "Number of devices found: " << nr_devices << std::endl;

            if (nr_devices < 1)
            {
                freenect_shutdown(f_ctx_);
                return false;
            }

            // Open device (like glview.c)
            if (freenect_open_device(f_ctx_, &f_dev_, 0) < 0)
            {
                std::cout << "Could not open device" << std::endl;
                freenect_shutdown(f_ctx_);
                return false;
            }

            // Allocate buffers (CRITICAL - like glview.c)
            allocateBuffers();

            // Set video modes (like glview.c)
            freenect_set_video_mode(f_dev_, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
            freenect_set_depth_mode(f_dev_, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));

            // Set callbacks (like glview.c)
            freenect_set_video_callback(f_dev_, rgb_cb);
            freenect_set_depth_callback(f_dev_, depth_cb);

            // CRITICAL: Set video buffer (like glview.c)
            freenect_set_video_buffer(f_dev_, rgb_back_);

            // Start streams (like glview.c)
            if (freenect_start_depth(f_dev_) < 0)
            {
                std::cout << "Failed to start depth stream" << std::endl;
                return false;
            }

            if (freenect_start_video(f_dev_) < 0)
            {
                std::cout << "Failed to start video stream" << std::endl;
                return false;
            }

            kinect_start_time_ = std::chrono::steady_clock::now();
            std::cout << "Kinect initialized successfully (glview.c style)" << std::endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cout << "Kinect initialization failed: " << e.what() << std::endl;
            return false;
        }
    }

    void allocateBuffers()
    {
        // Allocate buffers exactly like glview.c
        depth_mid_ = (uint16_t *)malloc(640 * 480 * sizeof(uint16_t));
        depth_front_ = (uint16_t *)malloc(640 * 480 * sizeof(uint16_t));
        rgb_back_ = (uint8_t *)malloc(640 * 480 * 3);
        rgb_mid_ = (uint8_t *)malloc(640 * 480 * 3);
        rgb_front_ = (uint8_t *)malloc(640 * 480 * 3);

        std::cout << "Buffers allocated (glview.c style)" << std::endl;
    }

    void startKinectThread()
    {
        // Start freenect thread exactly like glview.c freenect_threadfunc
        kinect_thread_ = std::thread([this]()
                                     {
            std::cout << "🔄 Starting Kinect thread (glview.c freenect_threadfunc style)" << std::endl;
            
            while (!die_ && freenect_process_events(f_ctx_) >= 0) {
                // Tight loop like glview.c - NO sleep!
                // This is the key difference from our previous approach
            }
            
            std::cout << "🔄 Kinect thread stopped" << std::endl; });
    }

    void cleanupKinect()
    {
        if (f_dev_)
        {
            freenect_stop_depth(f_dev_);
            freenect_stop_video(f_dev_);
            freenect_close_device(f_dev_);
        }

        if (f_ctx_)
        {
            freenect_shutdown(f_ctx_);
        }

        // Free buffers
        if (depth_mid_)
            free(depth_mid_);
        if (depth_front_)
            free(depth_front_);
        if (rgb_back_)
            free(rgb_back_);
        if (rgb_mid_)
            free(rgb_mid_);
        if (rgb_front_)
            free(rgb_front_);

        std::cout << "Kinect cleanup complete" << std::endl;
    }

    SimpleDetectionResult processFrames(const cv::Mat &rgb, const cv::Mat &depth, uint32_t timestamp)
    {
        SimpleDetectionResult result(timestamp);

        if (kinect_mode_)
        {
            // Real detection on Kinect data
            auto detection_result = detector_.detectObjects(rgb, depth);
            result.hands = detection_result.first;
            result.objects = detection_result.second;
        }
        else
        {
            // Dummy mode
            result = generateDummyDetections(timestamp);
        }

        return result;
    }

    // ========== DUMMY MODE ==========

    void setupDummyMode()
    {
        dummy_start_time_ = std::chrono::steady_clock::now();
    }

    void generateDummyFrames()
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - dummy_start_time_);
        uint32_t timestamp = static_cast<uint32_t>(elapsed.count());

        SimpleDetectionResult result = generateDummyDetections(timestamp);

        if (has_client_)
        {
            broadcastDetectionResult(result);
        }
    }

    SimpleDetectionResult generateDummyDetections(uint32_t timestamp)
    {
        SimpleDetectionResult result(timestamp);

        static float counter = 0.0f;
        counter += 0.1f; // Increment counter for dummy data

        // result.hands.emplace_back(
        //     std::sin(counter) * 0.05f + 0.05f,
        //     std::cos(counter) * 0.05f + 0.05f,
        //     0,
        //     0);

        // result.hands.emplace_back(
        //     1 - (std::sin(counter) * 0.05f + 0.05f),
        //     1 - (std::cos(counter) * 0.05f + 0.05f),
        //     0,
        //     1);

        result.hands.emplace_back(
            std::sin(counter) * 0.5f + 0.5f,
            std::cos(counter) * 0.5f + 0.5f,
            0, 0);
        result.hands.emplace_back(
            1 - (std::sin(counter) * 0.5f + 0.5f),
            1 - (std::cos(counter) * 0.5f + 0.5f),
            0, 1);

        // Dummy objects
        result.objects.emplace_back(0.2f, 0.5f, 0.85f, 1);
        result.objects.emplace_back(0.8f, 0.5f, 0.85f, 2);

        return result;
    }

    void broadcastDetectionResult(const SimpleDetectionResult &result)
    {
        if (!has_client_)
            return;

        std::string json = detectionResultToJSON(result);

        ssize_t bytes_sent = sendto(socket_fd_, json.c_str(), json.length(), 0,
                                    reinterpret_cast<const sockaddr *>(&client_addr_),
                                    sizeof(client_addr_));

        if (bytes_sent > 0)
        {
            message_count_++;
        }
    }

    std::string detectionResultToJSON(const SimpleDetectionResult &result)
    {
        std::ostringstream json;
        json << std::fixed << std::setprecision(3);

        json << "{";
        json << "\"timestamp\":" << result.timestamp << ",";
        json << "\"mode\":\"" << (kinect_mode_ ? "kinect" : "dummy") << "\",";

        json << "\"hands\":[";
        for (size_t i = 0; i < result.hands.size(); ++i)
        {
            const auto &hand = result.hands[i];
            json << "{\"x\":" << hand.x << ",\"y\":" << hand.y << ",\"z\":" << hand.z
                 << ",\"id\":" << hand.id << "}";
            if (i < result.hands.size() - 1)
                json << ",";
        }
        json << "],";

        json << "\"objects\":[";
        for (size_t i = 0; i < result.objects.size(); ++i)
        {
            const auto &object = result.objects[i];
            json << "{\"x\":" << object.x << ",\"y\":" << object.y << ",\"z\":" << object.z
                 << ",\"id\":" << object.id << "}";
            if (i < result.objects.size() - 1)
                json << ",";
        }
        json << "]";

        json << "}";

        return json.str();
    }

    // Network members
    int port_;
    std::string ip_;
    int socket_fd_;
    sockaddr_in server_addr_;
    sockaddr_in client_addr_;
    std::atomic<bool> running_{false};
    std::atomic<bool> has_client_{false};
    std::atomic<int> message_count_{0};

    // Kinect members (glview.c style)
    bool kinect_mode_;
    freenect_context *f_ctx_ = nullptr;
    freenect_device *f_dev_ = nullptr;
    std::thread kinect_thread_;
    std::atomic<bool> die_{false};

    // Buffers (exactly like glview.c)
    uint16_t *depth_mid_ = nullptr, *depth_front_ = nullptr;
    uint8_t *rgb_back_ = nullptr, *rgb_mid_ = nullptr, *rgb_front_ = nullptr;

    // Frame synchronization (glview.c style)
    std::mutex gl_backbuf_mutex_;
    std::atomic<bool> got_rgb_{false};
    std::atomic<bool> got_depth_{false};

    // Detection
    SimpleDetector detector_;

    // Timing
    std::chrono::steady_clock::time_point kinect_start_time_;
    std::chrono::steady_clock::time_point dummy_start_time_;
};

// Static instance for freenect callbacks (defined here since it's now public)
SimpleUDPServer *SimpleUDPServer::instance_ = nullptr;

// Global server for signal handling
std::unique_ptr<SimpleUDPServer> g_server;

void signalHandler(int signal)
{
    std::cout << "\nShutdown signal received" << std::endl;
    if (g_server)
    {
        g_server->stop();
    }
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    if (argc > 1) {
        int port = std::atoi(argv[1]);
    }

    std::cout << "Simple UDP Object Detection Server" << std::endl;
    std::cout << "Architecture: glview.c style (direct freenect integration)" << std::endl;
    
    g_server = std::make_unique<SimpleUDPServer>();
    
    // Set static instance for callbacks
    SimpleUDPServer::instance_ = g_server.get();

    if (!g_server->start())
    {
        std::cerr << "Failed to start server at ip " << std::endl;
        return 1;
    }

    g_server->runServerLoop();

    std::cout << "Server stopped" << std::endl;
    return 0;
}
