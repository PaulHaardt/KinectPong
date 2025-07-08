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

// Add Kinect integration
#include "capture-cv.hpp"

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

// Simple detection structures (KISS version)
struct SimpleDetectedObject {
    float x, y, z;
    std::string type;
    float confidence;
    
    SimpleDetectedObject(float x_, float y_, float z_, const std::string& type_, float conf_)
        : x(x_), y(y_), z(z_), type(type_), confidence(conf_) {}
};

struct SimpleDetectionResult {
    std::vector<SimpleDetectedObject> hands;
    std::vector<SimpleDetectedObject> objects;
    uint32_t timestamp;
    
    SimpleDetectionResult(uint32_t ts) : timestamp(ts) {}
};

class SimpleUDPServer {
public:
    SimpleUDPServer(int port = 8888) : port_(port), socket_fd_(-1), 
                                       capture_(nullptr),
                                       frame_sync_threshold_ms_(50),
                                       kinect_mode_(false) {
#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    }
    
    ~SimpleUDPServer() {
        stop();
#ifdef _WIN32
        WSACleanup();
#endif
    }
    
    bool start() {
        std::cout << "Starting UDP server..." << std::endl;
        
        // Start UDP server first
        if (!startUDPServer()) {
            return false;
        }
        
        // Try to start Kinect - if it fails, use dummy mode
        if (tryStartKinect()) {
            kinect_mode_ = true;
            std::cout << "🟢 KINECT MODE: Real sensor data" << std::endl;
        } else {
            kinect_mode_ = false;
            std::cout << "🟡 DUMMY MODE: Simulated sensor data (no Kinect detected)" << std::endl;
            setupDummyMode();
        }
        
        running_ = true;
        std::cout << "System ready!" << std::endl;
        return true;
    }
    
    void stop() {
        running_ = false;
        
        if (kinect_mode_ && capture_) {
            try {
                capture_->stop();
            } catch (const std::exception& e) {
                std::cerr << "Error stopping Kinect: " << e.what() << std::endl;
            }
        }
        
        if (socket_fd_ >= 0) {
#ifdef _WIN32
            closesocket(socket_fd_);
#else
            close(socket_fd_);
#endif
            socket_fd_ = -1;
        }
    }
    
    void runServerLoop() {
        char buffer[1024];
        sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        
        // Set receive timeout to make it non-blocking
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; // 10ms
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, 
                   reinterpret_cast<const char*>(&timeout), sizeof(timeout));
        
        std::cout << "Server listening... Send 'SUBSCRIBE' to receive data" << std::endl;
        
        auto last_dummy_time = std::chrono::steady_clock::now();
        
        while (running_) {
            // Handle Kinect events OR dummy timing
            if (kinect_mode_) {
                // Process Kinect events (CRITICAL: must be called frequently)
                try {
                    capture_->next_loop_event();
                } catch (const std::exception& e) {
                    std::cerr << "Kinect processing error: " << e.what() << std::endl;
                }
            } else {
                // Dummy mode: generate frames at ~30fps
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_dummy_time);
                
                if (elapsed.count() >= 33) {  // ~30fps
                    generateDummyFrames();
                    last_dummy_time = now;
                }
            }
            
            // Check for new client subscriptions
            ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                             reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
            
            if (bytes_received > 0) {
                buffer[bytes_received] = '\0';
                std::string message(buffer);
                
                if (message == "SUBSCRIBE") {
                    client_addr_ = client_addr;
                    has_client_ = true;
                    
                    std::cout << "Client subscribed: " << inet_ntoa(client_addr.sin_addr) 
                              << ":" << ntohs(client_addr.sin_port) << std::endl;
                    
                    std::string ack_msg = kinect_mode_ ? 
                        "{\"status\":\"subscribed\", \"mode\":\"kinect\"}" :
                        "{\"status\":\"subscribed\", \"mode\":\"dummy\"}";
                        
                    sendto(socket_fd_, ack_msg.c_str(), ack_msg.length(), 0,
                           reinterpret_cast<const sockaddr*>(&client_addr), sizeof(client_addr));
                }
            }
            
            // Small sleep to prevent CPU spinning
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

private:
    bool startUDPServer() {
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }
        
        int opt = 1;
        setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, 
                   reinterpret_cast<const char*>(&opt), sizeof(opt));
        
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_addr.s_addr = INADDR_ANY;
        server_addr_.sin_port = htons(port_);
        
        if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_)) < 0) {
            std::cerr << "Failed to bind to port " << port_ << std::endl;
            return false;
        }
        
        std::cout << "UDP Server started on port " << port_ << std::endl;
        return true;
    }
    
    bool tryStartKinect() {
        try {
            std::cout << "Attempting to connect to Kinect..." << std::endl;
            
            // Create capture instance
            capture_ = std::make_unique<CVKinectCapture>(CVKinectCapture::MEDIUM, CVKinectCapture::MEDIUM);
            
            // Setup callbacks
            setupKinectCallbacks();
            
            // Try to start
            capture_->start();
            
            std::cout << "Kinect connected successfully!" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "Kinect not available: " << e.what() << std::endl;
            capture_.reset();
            return false;
        }
    }
    
    void setupKinectCallbacks() {
        if (!capture_) return;
        
        // RGB frame callback (using existing capture-cv.hpp pattern)
        capture_->set_rgb_callback([this](cv::Mat& rgb, uint32_t timestamp) {
            onRGBFrame(rgb, timestamp);
        });
        
        // Depth frame callback
        capture_->set_depth_callback([this](cv::Mat& depth, uint32_t timestamp) {
            onDepthFrame(depth, timestamp);
        });
    }
    
    void setupDummyMode() {
        std::cout << "Setting up dummy data generation..." << std::endl;
        dummy_start_time_ = std::chrono::steady_clock::now();
    }
    
    void generateDummyFrames() {
        // Generate dummy RGB and depth frames with realistic timing
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - dummy_start_time_);
        uint32_t timestamp = static_cast<uint32_t>(elapsed.count());
        
        // Create dummy RGB frame (empty for processing, we don't actually need the pixels)
        cv::Mat dummy_rgb = cv::Mat::zeros(480, 640, CV_8UC3);
        
        // Create dummy depth frame (empty for processing, we don't actually need the pixels)
        cv::Mat dummy_depth = cv::Mat::zeros(480, 640, CV_16UC1);
        
        // Process as if they were real frames
        onRGBFrame(dummy_rgb, timestamp);
        onDepthFrame(dummy_depth, timestamp + 2);  // Slight offset to test sync
    }
    
    void onRGBFrame(cv::Mat& rgb, uint32_t timestamp) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_rgb_ = rgb.clone();
        latest_rgb_timestamp_ = timestamp;
        checkAndProcessFrames();
        
        // Debug output (reduce spam)
        if (rgb_frame_count_++ % 30 == 0) {
            std::string mode = kinect_mode_ ? "[KINECT]" : "[DUMMY]";
            std::cout << mode << " RGB frame " << rgb_frame_count_ << " at " << timestamp << std::endl;
        }
    }
    
    void onDepthFrame(cv::Mat& depth, uint32_t timestamp) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_depth_ = depth.clone();
        latest_depth_timestamp_ = timestamp;
        checkAndProcessFrames();
        
        // Debug output (reduce spam)
        if (depth_frame_count_++ % 30 == 0) {
            std::string mode = kinect_mode_ ? "[KINECT]" : "[DUMMY]";
            std::cout << mode << " Depth frame " << depth_frame_count_ << " at " << timestamp << std::endl;
        }
    }
    
    void checkAndProcessFrames() {
        // Frame synchronization logic (from existing object_detection_server.cpp)
        if (!latest_rgb_.empty() && !latest_depth_.empty()) {
            uint32_t time_diff = std::abs(static_cast<int32_t>(latest_rgb_timestamp_ - latest_depth_timestamp_));
            
            if (time_diff <= frame_sync_threshold_ms_) {
                // Frames are synchronized! Process them.
                uint32_t sync_timestamp = std::max(latest_rgb_timestamp_, latest_depth_timestamp_);
                
                // Process frames (dummy detection for now, but real timing)
                SimpleDetectionResult result = processFrames(latest_rgb_, latest_depth_, sync_timestamp);
                
                // Broadcast to client if connected
                if (has_client_) {
                    broadcastDetectionResult(result);
                }
                
                // Clear processed frames
                latest_rgb_ = cv::Mat();
                latest_depth_ = cv::Mat();
                
                // Debug sync
                if (sync_count_++ % 30 == 0) {
                    std::string mode = kinect_mode_ ? "[KINECT]" : "[DUMMY]";
                    std::cout << mode << " Synchronized frame pair " << sync_count_ 
                              << " (time_diff: " << time_diff << "ms)" << std::endl;
                }
            } else {
                // Frames out of sync
                if (out_of_sync_count_++ % 100 == 0) {
                    std::cout << "Warning: frames out of sync by " << time_diff << "ms" << std::endl;
                }
            }
        }
    }
    
    SimpleDetectionResult processFrames(const cv::Mat& rgb, const cv::Mat& depth, uint32_t timestamp) {
        SimpleDetectionResult result(timestamp);
        
        // KISS: For now, return dummy data but with real timestamps
        // In dummy mode, make the animation more obvious
        // In Kinect mode, use same dummy data (until next iteration adds real detection)
        
        static int counter = 0;
        counter++;
        
        // Different animation patterns for Kinect vs Dummy mode (just for clarity)
        float speed_multiplier = kinect_mode_ ? 1.0f : 2.0f;  // Dummy mode animates faster
        
        if (counter % 60 < 45) {  // Show hand ~75% of the time
            result.hands.emplace_back(
                0.2f + 0.3f * sin(counter * 0.05f * speed_multiplier),  // x
                0.1f + 0.2f * cos(counter * 0.05f * speed_multiplier),  // y 
                1.0f + 0.3f * sin(counter * 0.03f * speed_multiplier),  // z
                "hand",
                kinect_mode_ ? 0.8f : 0.6f  // Lower confidence in dummy mode
            );
        }
        
        // Objects on table
        result.objects.emplace_back(0.1f, 0.5f, 0.8f, "object", kinect_mode_ ? 0.7f : 0.5f);
        if (counter % 40 < 30) {
            result.objects.emplace_back(-0.2f, 0.6f, 0.85f, "object", kinect_mode_ ? 0.6f : 0.4f);
        }
        
        return result;
    }
    
    void broadcastDetectionResult(const SimpleDetectionResult& result) {
        if (!has_client_) return;
        
        std::string json = detectionResultToJSON(result);
        
        ssize_t bytes_sent = sendto(socket_fd_, json.c_str(), json.length(), 0,
                                   reinterpret_cast<const sockaddr*>(&client_addr_), 
                                   sizeof(client_addr_));
        
        if (bytes_sent > 0) {
            message_count_++;
            if (message_count_ % 30 == 0) {
                std::string mode = kinect_mode_ ? "[KINECT]" : "[DUMMY]";
                std::cout << mode << " Sent " << message_count_ << " detection messages" << std::endl;
            }
        }
    }
    
    std::string detectionResultToJSON(const SimpleDetectionResult& result) {
        std::ostringstream json;
        json << std::fixed << std::setprecision(3);
        
        json << "{";
        json << "\"timestamp\":" << result.timestamp << ",";
        json << "\"mode\":\"" << (kinect_mode_ ? "kinect" : "dummy") << "\",";
        
        // Hands array (using existing format)
        json << "\"hands\":[";
        for (size_t i = 0; i < result.hands.size(); ++i) {
            const auto& hand = result.hands[i];
            json << "{\"x\":" << hand.x << ",\"y\":" << hand.y << ",\"z\":" << hand.z 
                 << ",\"confidence\":" << hand.confidence << "}";
            if (i < result.hands.size() - 1) json << ",";
        }
        json << "],";
        
        // Objects array (using existing format)
        json << "\"objects\":[";
        for (size_t i = 0; i < result.objects.size(); ++i) {
            const auto& object = result.objects[i];
            json << "{\"x\":" << object.x << ",\"y\":" << object.y << ",\"z\":" << object.z 
                 << ",\"type\":\"" << object.type << "\",\"confidence\":" << object.confidence << "}";
            if (i < result.objects.size() - 1) json << ",";
        }
        json << "]";
        
        json << "}";
        
        return json.str();
    }
    
    // Network members
    int port_;
    int socket_fd_;
    sockaddr_in server_addr_;
    sockaddr_in client_addr_;
    std::atomic<bool> running_{false};
    std::atomic<bool> has_client_{false};
    std::atomic<int> message_count_{0};
    
    // Kinect integration (now optional)
    std::unique_ptr<CVKinectCapture> capture_;
    bool kinect_mode_;
    
    // Frame synchronization (from existing code)
    std::mutex frame_mutex_;
    cv::Mat latest_rgb_;
    cv::Mat latest_depth_;
    uint32_t latest_rgb_timestamp_ = 0;
    uint32_t latest_depth_timestamp_ = 0;
    uint32_t frame_sync_threshold_ms_;
    
    // Dummy mode timing
    std::chrono::steady_clock::time_point dummy_start_time_;
    
    // Debug counters
    std::atomic<int> rgb_frame_count_{0};
    std::atomic<int> depth_frame_count_{0};
    std::atomic<int> sync_count_{0};
    std::atomic<int> out_of_sync_count_{0};
};

// Global server for signal handling
std::unique_ptr<SimpleUDPServer> g_server;

void signalHandler(int signal) {
    std::cout << "\nShutdown signal received" << std::endl;
    if (g_server) {
        g_server->stop();
    }
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    int port = 8888;
    if (argc > 1) {
        port = std::atoi(argv[1]);
    }
    
    std::cout << "Simple UDP Object Detection Server" << std::endl;
    std::cout << "Will auto-detect Kinect or fall back to dummy mode" << std::endl;
    std::cout << "Starting on port " << port << std::endl;
    
    g_server = std::make_unique<SimpleUDPServer>(port);
    
    if (!g_server->start()) {
        std::cerr << "Failed to start server" << std::endl;
        return 1;
    }
    
    g_server->runServerLoop();
    
    std::cout << "Server stopped" << std::endl;
    return 0;
}