#include "udp_json_server.hpp"
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <algorithm>
#include <iomanip>

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")
#endif

UDPJSONServer::UDPJSONServer(int port) : port_(port) {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

UDPJSONServer::~UDPJSONServer() {
    stop();
#ifdef _WIN32
    WSACleanup();
#endif
}

bool UDPJSONServer::start() {
    if (running_) {
        return true;
    }
    
    // Create UDP socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }
    
    // Set socket options
    int opt = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, 
                   reinterpret_cast<const char*>(&opt), sizeof(opt)) < 0) {
        std::cerr << "Failed to set socket options" << std::endl;
#ifdef _WIN32
        closesocket(socket_fd_);
#else
        close(socket_fd_);
#endif
        return false;
    }
    
    // Bind socket
    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_addr.s_addr = INADDR_ANY;
    server_addr_.sin_port = htons(port_);
    
    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_)) < 0) {
        std::cerr << "Failed to bind socket to port " << port_ << std::endl;
#ifdef _WIN32
        closesocket(socket_fd_);
#else
        close(socket_fd_);
#endif
        return false;
    }
    
    running_ = true;
    server_thread_ = std::thread(&UDPJSONServer::serverLoop, this);
    
    std::cout << "UDP JSON Server started on port " << port_ << std::endl;
    return true;
}

void UDPJSONServer::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    
    if (socket_fd_ >= 0) {
#ifdef _WIN32
        closesocket(socket_fd_);
#else
        close(socket_fd_);
#endif
        socket_fd_ = -1;
    }
    
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        clients_.clear();
    }
    
    std::cout << "UDP JSON Server stopped" << std::endl;
}

void UDPJSONServer::broadcastDetectionResult(const DetectionResult& result) {
    std::string json_message = detectionResultToJSON(result);
    
    {
        std::lock_guard<std::mutex> lock(message_queue_mutex_);
        message_queue_.push(json_message);
    }
}

void UDPJSONServer::serverLoop() {
    char buffer[1024];
    sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    // Set socket timeout for non-blocking operations
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100ms
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, 
               reinterpret_cast<const char*>(&timeout), sizeof(timeout));
    
    auto last_cleanup = std::chrono::steady_clock::now();
    
    while (running_) {
        // Handle incoming messages (client registration)
        handleIncomingMessages();
        
        // Process message queue
        std::queue<std::string> messages_to_send;
        {
            std::lock_guard<std::mutex> lock(message_queue_mutex_);
            messages_to_send.swap(message_queue_);
        }
        
        while (!messages_to_send.empty()) {
            const std::string& message = messages_to_send.front();
            
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                for (const auto& client : clients_) {
                    ssize_t bytes_sent = sendto(socket_fd_, message.c_str(), message.length(), 0,
                                              reinterpret_cast<const sockaddr*>(&client.address),
                                              sizeof(client.address));
                    if (bytes_sent > 0) {
                        bytes_transmitted_ += bytes_sent;
                    }
                }
                if (!clients_.empty()) {
                    messages_sent_++;
                }
            }
            
            messages_to_send.pop();
        }
        
        // Cleanup inactive clients periodically
        auto now = std::chrono::steady_clock::now();
        if (now - last_cleanup > std::chrono::seconds(10)) {
            cleanupInactiveClients();
            last_cleanup = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void UDPJSONServer::handleIncomingMessages() {
    char buffer[1024];
    sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                     reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
    
    if (bytes_received > 0) {
        buffer[bytes_received] = '\0';
        std::string message(buffer);
        
        // Simple protocol: "SUBSCRIBE" to register for updates
        if (message == "SUBSCRIBE") {
            addClient(client_addr);
            
            // Send acknowledgment
            std::string ack = "{\"status\":\"subscribed\"}";
            sendto(socket_fd_, ack.c_str(), ack.length(), 0,
                   reinterpret_cast<const sockaddr*>(&client_addr), sizeof(client_addr));
        }
    }
}

void UDPJSONServer::addClient(const sockaddr_in& client_addr) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    
    // Check if client already exists
    for (auto& client : clients_) {
        if (client.address.sin_addr.s_addr == client_addr.sin_addr.s_addr &&
            client.address.sin_port == client_addr.sin_port) {
            // Update last seen time
            client.last_seen = std::chrono::steady_clock::now();
            return;
        }
    }
    
    // Add new client if under limit
    if (clients_.size() < max_clients_) {
        clients_.emplace_back(client_addr);
        std::cout << "New client connected: " << inet_ntoa(client_addr.sin_addr) 
                  << ":" << ntohs(client_addr.sin_port) << std::endl;
    }
}

void UDPJSONServer::cleanupInactiveClients() {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto it = clients_.begin();
    
    while (it != clients_.end()) {
        if (now - it->last_seen > client_timeout_) {
            std::cout << "Removing inactive client: " << inet_ntoa(it->address.sin_addr) 
                      << ":" << ntohs(it->address.sin_port) << std::endl;
            it = clients_.erase(it);
        } else {
            ++it;
        }
    }
}

size_t UDPJSONServer::getClientCount() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    return clients_.size();
}

std::string UDPJSONServer::detectionResultToJSON(const DetectionResult& result) {
    std::ostringstream json;
    json << std::fixed << std::setprecision(3);
    
    json << "{";
    json << "\"timestamp\":" << result.timestamp << ",";
    
    // Hands array
    json << "\"hands\":[";
    for (size_t i = 0; i < result.hands.size(); ++i) {
        const auto& hand = result.hands[i];
        json << "{\"x\":" << hand.x << ",\"y\":" << hand.y << ",\"z\":" << hand.z 
             << ",\"confidence\":" << hand.confidence << "}";
        if (i < result.hands.size() - 1) json << ",";
    }
    json << "],";
    
    // Objects array
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