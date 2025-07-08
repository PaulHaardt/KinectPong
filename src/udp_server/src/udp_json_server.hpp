#pragma once

#include "object_detector.hpp"
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

struct UDPClient {
    sockaddr_in address;
    std::chrono::steady_clock::time_point last_seen;
    
    UDPClient(const sockaddr_in& addr) : address(addr) {
        last_seen = std::chrono::steady_clock::now();
    }
    
    bool operator==(const UDPClient& other) const {
        return address.sin_addr.s_addr == other.address.sin_addr.s_addr &&
               address.sin_port == other.address.sin_port;
    }
};

class UDPJSONServer {
public:
    UDPJSONServer(int port = 8888);
    ~UDPJSONServer();
    
    // Server control
    bool start();
    void stop();
    bool isRunning() const { return running_; }
    
    // Data transmission
    void broadcastDetectionResult(const DetectionResult& result);
    void setMaxClients(size_t max_clients) { max_clients_ = max_clients; }
    
    // Statistics
    size_t getClientCount() const;
    size_t getMessagesSent() const { return messages_sent_; }
    size_t getBytesTransmitted() const { return bytes_transmitted_; }

private:
    // Network operations
    void serverLoop();
    void handleIncomingMessages();
    void cleanupInactiveClients();
    std::string detectionResultToJSON(const DetectionResult& result);
    
    // Client management
    void addClient(const sockaddr_in& client_addr);
    void removeClient(const sockaddr_in& client_addr);
    bool isKnownClient(const sockaddr_in& client_addr);
    
    // Server configuration
    int port_;
    size_t max_clients_ = 100;
    std::chrono::seconds client_timeout_{30};
    
    // Network state
    int socket_fd_ = -1;
    sockaddr_in server_addr_;
    std::atomic<bool> running_{false};
    
    // Client tracking
    mutable std::mutex clients_mutex_;
    std::vector<UDPClient> clients_;
    
    // Threading
    std::thread server_thread_;
    
    // Statistics
    std::atomic<size_t> messages_sent_{0};
    std::atomic<size_t> bytes_transmitted_{0};
    
    // Message queue for thread-safe communication
    mutable std::mutex message_queue_mutex_;
    std::queue<std::string> message_queue_;
};