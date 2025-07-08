#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <string.h>
#include <iomanip>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

class UDPTestClient {
public:
    UDPTestClient(const std::string& server_ip = "127.0.0.1", int server_port = 8888)
        : server_ip_(server_ip), server_port_(server_port), socket_fd_(-1) {
#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    }
    
    ~UDPTestClient() {
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
            std::cerr << "Invalid server IP address: " << server_ip_ << std::endl;
            return false;
        }
        
        // Send subscription message
        std::string subscribe_msg = "SUBSCRIBE";
        ssize_t bytes_sent = sendto(socket_fd_, subscribe_msg.c_str(), subscribe_msg.length(), 0,
                                   reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_));
        
        if (bytes_sent < 0) {
            std::cerr << "Failed to send subscription message" << std::endl;
            return false;
        }
        
        // Set socket timeout for receiving
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, 
                  reinterpret_cast<const char*>(&timeout), sizeof(timeout));
        
        std::cout << "Connected to server " << server_ip_ << ":" << server_port_ << std::endl;
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
        
        size_t message_count = 0;
        auto start_time = std::chrono::steady_clock::now();
        
        std::cout << "Listening for object detection data..." << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;
        
        while (running_) {
            ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                             reinterpret_cast<sockaddr*>(&sender_addr), &addr_len);
            
            if (bytes_received > 0) {
                buffer[bytes_received] = '\0';
                std::string message(buffer);
                
                message_count++;
                
                // Parse and display the JSON message
                std::cout << "\n--- Message #" << message_count << " ---" << std::endl;
                std::cout << message << std::endl;
                
                // Calculate message rate
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
                if (elapsed.count() > 0) {
                    double rate = static_cast<double>(message_count) / elapsed.count();
                    std::cout << "Message rate: " << std::fixed << std::setprecision(1) << rate << " msg/s" << std::endl;
                }
            } else if (bytes_received == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
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

// Global client instance for signal handling
std::unique_ptr<UDPTestClient> g_client;

void signalHandler(int signal) {
    std::cout << "\nShutdown signal received" << std::endl;
    if (g_client) {
        g_client->stop();
    }
}

int main(int argc, char* argv[]) {
    std::string server_ip = "127.0.0.1";
    int server_port = 8888;
    
    // Parse command line arguments
    if (argc > 1) {
        server_ip = argv[1];
    }
    if (argc > 2) {
        try {
            server_port = std::stoi(argv[2]);
        } catch (const std::exception& e) {
            std::cerr << "Invalid port number: " << argv[2] << std::endl;
            return 1;
        }
    }
    
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "UDP Object Detection Test Client" << std::endl;
    std::cout << "Connecting to: " << server_ip << ":" << server_port << std::endl;
    
    g_client = std::make_unique<UDPTestClient>(server_ip, server_port);
    
    if (!g_client->connect()) {
        std::cerr << "Failed to connect to server" << std::endl;
        return 1;
    }
    
    // Listen for messages
    g_client->listenForMessages();
    
    g_client->disconnect();
    g_client.reset();
    
    std::cout << "Client shutdown complete" << std::endl;
    return 0;
}