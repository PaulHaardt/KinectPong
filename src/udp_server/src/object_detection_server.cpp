#include <iostream>
#include <chrono>
#include <csignal>
#include <thread>
#include <atomic>
#include <memory>

#include "capture-cv.hpp"
#include "object_detector.hpp"
#include "udp_json_server.hpp"

class ObjectDetectionServer {
public:
    ObjectDetectionServer(int udp_port = 8888)
        : detector_(), udp_server_(udp_port), capture_(CVKinectCapture::MEDIUM, CVKinectCapture::MEDIUM) {
        
        // Initialize frame synchronization
        latest_rgb_timestamp_ = 0;
        latest_depth_timestamp_ = 0;
        frame_sync_threshold_ms_ = 50; // Allow 50ms difference between RGB and depth
    }
    
    bool start() {
        // Start UDP server
        if (!udp_server_.start()) {
            std::cerr << "Failed to start UDP server" << std::endl;
            return false;
        }
        
        // Configure object detector
        detector_.setHandDetectionEnabled(true);
        detector_.setObjectDetectionEnabled(true);
        detector_.setDepthRange(500.0f, 3000.0f); // 0.5m to 3m range
        
        // Set up Kinect callbacks
        capture_.set_rgb_callback([this](cv::Mat& rgb, uint32_t timestamp) {
            onRGBFrame(rgb, timestamp);
        });
        
        capture_.set_depth_callback([this](cv::Mat& depth, uint32_t timestamp) {
            onDepthFrame(depth, timestamp);
        });
        
        try {
            capture_.start();
            std::cout << "Kinect capture started" << std::endl;
            
            // Start processing thread
            running_ = true;
            processing_thread_ = std::thread(&ObjectDetectionServer::processingLoop, this);
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to start Kinect: " << e.what() << std::endl;
            udp_server_.stop();
            return false;
        }
    }
    
    void stop() {
        running_ = false;
        
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        
        try {
            capture_.stop();
        } catch (const std::exception& e) {
            std::cerr << "Error stopping Kinect: " << e.what() << std::endl;
        }
        
        udp_server_.stop();
        std::cout << "Object detection server stopped" << std::endl;
    }
    
    void printStatistics() {
        std::cout << "\n=== Server Statistics ===" << std::endl;
        std::cout << "Connected clients: " << udp_server_.getClientCount() << std::endl;
        std::cout << "Messages sent: " << udp_server_.getMessagesSent() << std::endl;
        std::cout << "Bytes transmitted: " << udp_server_.getBytesTransmitted() << std::endl;
        std::cout << "Frames processed: " << frames_processed_ << std::endl;
        std::cout << "Detection rate: " << detection_rate_fps_ << " FPS" << std::endl;
        std::cout << "=========================" << std::endl;
    }

private:
    void onRGBFrame(cv::Mat& rgb, uint32_t timestamp) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_rgb_ = rgb.clone();
        latest_rgb_timestamp_ = timestamp;
        checkAndProcessFrames();
    }
    
    void onDepthFrame(cv::Mat& depth, uint32_t timestamp) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_depth_ = depth.clone();
        latest_depth_timestamp_ = timestamp;
        checkAndProcessFrames();
    }
    
    void checkAndProcessFrames() {
        // Check if we have both RGB and depth frames with similar timestamps
        if (!latest_rgb_.empty() && !latest_depth_.empty()) {
            uint32_t time_diff = std::abs(static_cast<int32_t>(latest_rgb_timestamp_ - latest_depth_timestamp_));
            
            if (time_diff <= frame_sync_threshold_ms_) {
                // Frames are synchronized, add to processing queue
                FramePair frame_pair;
                frame_pair.rgb = latest_rgb_.clone();
                frame_pair.depth = latest_depth_.clone();
                frame_pair.timestamp = std::max(latest_rgb_timestamp_, latest_depth_timestamp_);
                
                {
                    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
                    if (frame_queue_.size() < max_queue_size_) {
                        frame_queue_.push(frame_pair);
                    }
                }
                
                // Clear processed frames
                latest_rgb_ = cv::Mat();
                latest_depth_ = cv::Mat();
            }
        }
    }
    
    void processingLoop() {
        auto last_stats_time = std::chrono::steady_clock::now();
        size_t frames_since_last_stats = 0;
        
        while (running_) {
            // Process Kinect events
            try {
                capture_.next_loop_event();
            } catch (const std::exception& e) {
                std::cerr << "Kinect processing error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            // Process frame queue
            FramePair frame_pair;
            bool has_frame = false;
            
            std::lock_guard<std::mutex> queue_lock(queue_mutex_);
            if (!frame_queue_.empty()) {
                frame_pair = frame_queue_.front();
                frame_queue_.pop();
                has_frame = true;
            }
            
            if (has_frame) {
                // Perform object detection
                DetectionResult result = detector_.detectObjects(frame_pair.rgb, frame_pair.depth, frame_pair.timestamp);
                
                // Broadcast results
                udp_server_.broadcastDetectionResult(result);
                
                frames_processed_++;
                frames_since_last_stats++;
                
                // Update statistics every 5 seconds
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_time);
                
                if (elapsed.count() >= 5000) {
                    detection_rate_fps_ = frames_since_last_stats * 1000.0f / elapsed.count();
                    frames_since_last_stats = 0;
                    last_stats_time = now;
                }
            }
            else {
                std::cout << "No frames to process, waiting..." << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    
    struct FramePair {
        cv::Mat rgb;
        cv::Mat depth;
        uint32_t timestamp;
    };
    
    // Core components
    ObjectDetector detector_;
    UDPJSONServer udp_server_;
    CVKinectCapture capture_;
    
    // Frame synchronization
    std::mutex frame_mutex_;
    cv::Mat latest_rgb_;
    cv::Mat latest_depth_;
    uint32_t latest_rgb_timestamp_;
    uint32_t latest_depth_timestamp_;
    uint32_t frame_sync_threshold_ms_;
    
    // Processing queue
    std::mutex queue_mutex_;
    std::queue<FramePair> frame_queue_;
    static const size_t max_queue_size_ = 5;
    
    // Threading
    std::atomic<bool> running_{false};
    std::thread processing_thread_;
    
    // Statistics
    std::atomic<size_t> frames_processed_{0};
    std::atomic<float> detection_rate_fps_{0.0f};
};

// Global server instance for signal handling
std::unique_ptr<ObjectDetectionServer> g_server;
std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int signal) {
    std::cout << "\nShutdown signal received (" << signal << ")" << std::endl;
    g_shutdown_requested = true;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    int udp_port = 8888;
    if (argc > 1) {
        try {
            udp_port = std::stoi(argv[1]);
        } catch (const std::exception& e) {
            std::cerr << "Invalid port number: " << argv[1] << std::endl;
            return 1;
        }
    }
    
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "Starting Object Detection UDP Server..." << std::endl;
    std::cout << "UDP Port: " << udp_port << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    
    // Create and start server
    g_server = std::make_unique<ObjectDetectionServer>(udp_port);
    
    if (!g_server->start()) {
        std::cerr << "Failed to start server" << std::endl;
        return 1;
    }
    
    std::cout << "Server started successfully!" << std::endl;
    std::cout << "Clients can connect by sending 'SUBSCRIBE' to UDP port " << udp_port << std::endl;
    
    // Main loop
    auto last_stats_print = std::chrono::steady_clock::now();
    
    while (!g_shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Print statistics every 30 seconds
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_print).count() >= 2) {
            g_server->printStatistics();
            last_stats_print = now;
        }
    }
    
    std::cout << "Shutting down server..." << std::endl;
    g_server->stop();
    g_server.reset();
    
    std::cout << "Server shutdown complete" << std::endl;
    return 0;
}