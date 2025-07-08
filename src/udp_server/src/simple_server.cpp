#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <sstream>
#include <atomic>
#include <csignal>
#include <math.h>

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

class SimpleUDPServer {
public:
    SimpleUDPServer(int port = 8888) : port_(port), socket_fd_(-1) {
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
        // Create UDP socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }
        
        // Set socket reuse
        int opt = 1;
        setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, 
                   reinterpret_cast<const char*>(&opt), sizeof(opt));
        
        // Bind socket
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_addr.s_addr = INADDR_ANY;
        server_addr_.sin_port = htons(port_);
        
        if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_)) < 0) {
            std::cerr << "Failed to bind to port " << port_ << std::endl;
            return false;
        }
        
        running_ = true;
        std::cout << "UDP Server started on port " << port_ << std::endl;
        return true;
    }
    
    void stop() {
        running_ = false;
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
        timeout.tv_usec = 100000; // 100ms
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, 
                   reinterpret_cast<const char*>(&timeout), sizeof(timeout));
        
        std::cout << "Server listening... Send 'SUBSCRIBE' to receive data" << std::endl;
        
        while (running_) {
            // Check for new client subscriptions
            ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                             reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
            
            if (bytes_received > 0) {
                buffer[bytes_received] = '\0';
                std::string message(buffer);
                
                if (message == "SUBSCRIBE") {
                    // Add client to our list (simple version - just store the last one)
                    client_addr_ = client_addr;
                    has_client_ = true;
                    
                    std::cout << "Client subscribed: " << inet_ntoa(client_addr.sin_addr) 
                              << ":" << ntohs(client_addr.sin_port) << std::endl;
                    
                    // Send acknowledgment
                    std::string ack = "{\"status\":\"subscribed\"}";
                    sendto(socket_fd_, ack.c_str(), ack.length(), 0,
                           reinterpret_cast<const sockaddr*>(&client_addr), sizeof(client_addr));
                }
            }
            
            // Broadcast dummy data every 100ms if we have a client
            if (has_client_) {
                broadcastDummyData();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
private:
    void broadcastDummyData() {
        if (!has_client_) return;
        
        // Create dummy JSON data
        std::string json = createDummyJSON();
        
        // Send to client
        ssize_t bytes_sent = sendto(socket_fd_, json.c_str(), json.length(), 0,
                                   reinterpret_cast<const sockaddr*>(&client_addr_), 
                                   sizeof(client_addr_));
        
        if (bytes_sent > 0) {
            message_count_++;
            if (message_count_ % 10 == 0) {  // Print every 10 messages
                std::cout << "Sent " << message_count_ << " messages" << std::endl;
            }
        }
    }
    
    std::string createDummyJSON() {
        static int counter = 0;
        counter++;
        
        // Create dummy hand and object coordinates
        std::ostringstream json;
        json << "{";
        json << "\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count() << ",";
        
        // Dummy hands (simulate 1-2 hands moving)
        json << "\"hands\":[";
        json << "[" << 0.5 + 0.3 * sin(counter * 0.1) << ","
             << 0.3 + 0.2 * cos(counter * 0.1) << ","
             << 1.2 << "]";
        if (counter % 20 < 15) {  // Sometimes show second hand
            json << ",[" << -0.3 + 0.2 * sin(counter * 0.15) << ","
                 << 0.4 + 0.1 * cos(counter * 0.15) << ","
                 << 1.1 << "]";
        }
        json << "],";
        
        // Dummy objects (simulate 2-3 objects on table)
        json << "\"objects\":[";
        json << "[0.1,0.8,0.9]";
        json << ",[-0.2,0.7,0.85]";
        if (counter % 30 < 20) {  // Sometimes show third object
            json << ",[0.4,0.6,0.95]";
        }
        json << "]";
        
        json << "}";
        
        return json.str();
    }
    
    int port_;
    int socket_fd_;
    sockaddr_in server_addr_;
    sockaddr_in client_addr_;
    std::atomic<bool> running_{false};
    std::atomic<bool> has_client_{false};
    std::atomic<int> message_count_{0};
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