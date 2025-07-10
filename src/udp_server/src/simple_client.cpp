#include "utils.hpp"
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

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

class SimpleUDPClient {
public:
    SimpleUDPClient() : socket_fd_(-1) {
#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
        auto env = load_env(".env");
        server_port_ = std::stoi(env["UDP_SERVER_PORT"]);
        server_ip_ = env["UDP_IP_UBUNTU"];
        std::cout << "Connecting to: " << server_ip_ << ":" << server_port_ << std::endl;
    }
    
    ~SimpleUDPClient() {
        disconnect();
#ifdef _WIN32
        WSACleanup();
#endif
    }
    
    bool connect() {
        // Create UDP socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }
        
        // Set up server address
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(server_port_);
        
        if (inet_aton(server_ip_.c_str(), &server_addr_.sin_addr) == 0) {
            std::cerr << "Invalid server IP: " << server_ip_ << std::endl;
            return false;
        }
        
        // Send subscription message
        std::string subscribe_msg = "SUBSCRIBE";
        ssize_t bytes_sent = sendto(socket_fd_, subscribe_msg.c_str(), subscribe_msg.length(), 0,
                                   reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_));
        
        if (bytes_sent < 0) {
            std::cerr << "Failed to send subscription" << std::endl;
            return false;
        }
        
        // Set receive timeout
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, 
                  reinterpret_cast<const char*>(&timeout), sizeof(timeout));
        
        std::cout << "Connected to " << server_ip_ << ":" << server_port_ << std::endl;
        return true;
    }
    
    void disconnect() {
        if (socket_fd_ >= 0) {
#ifdef _WIN32
            closesocket(socket_fd_);
#else
            close(socket_fd_);
#endif
            socket_fd_ = -1;
        }
    }
    
    void listenForMessages() {
        char buffer[4096];
        sockaddr_in sender_addr;
        socklen_t addr_len = sizeof(sender_addr);
        
        int message_count = 0;
        auto start_time = std::chrono::steady_clock::now();
        
        std::cout << "Listening for object detection data..." << std::endl;
        std::cout << "Press Ctrl+C to stop\n" << std::endl;
        
        while (running_) {
            ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                             reinterpret_cast<sockaddr*>(&sender_addr), &addr_len);
            
            if (bytes_received > 0) {
                buffer[bytes_received] = '\0';
                std::string message(buffer);
                
                message_count++;
                
                // Pretty print every 10th message to avoid spam
                if (message_count % 10 == 1) {
                    std::cout << "--- Message #" << message_count << " ---" << std::endl;
                    std::cout << message << std::endl;
                    
                    // Calculate message rate
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
                    if (elapsed.count() > 0) {
                        double rate = static_cast<double>(message_count) / elapsed.count();
                        std::cout << "Rate: " << rate << " msg/s\n" << std::endl;
                    }
                } else {
                    // Just show a dot for other messages
                    std::cout << "." << std::flush;
                    if (message_count % 50 == 0) std::cout << std::endl;
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        std::cout << "\nReceived " << message_count << " messages total" << std::endl;
    }
    
    void stop() {
        running_ = false;
    }
    
private:
    std::string server_ip_;
    int server_port_;
    int socket_fd_;
    sockaddr_in server_addr_;
    std::atomic<bool> running_{true};
};

// Global client for signal handling
std::unique_ptr<SimpleUDPClient> g_client;

void signalHandler(int signal) {
    std::cout << "\nShutdown signal received" << std::endl;
    if (g_client) {
        g_client->stop();
    }
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "Simple UDP Test Client" << std::endl;
    
    g_client = std::make_unique<SimpleUDPClient>();
    
    if (!g_client->connect()) {
        std::cerr << "Failed to connect to server" << std::endl;
        return 1;
    }
    
    g_client->listenForMessages();
    
    g_client.reset();
    std::cout << "Client shutdown complete" << std::endl;
    return 0;
}
