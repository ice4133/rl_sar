// SPDX-License-Identifier: Apache-2.0
#include "transport.hpp"
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace sim_tuner {
namespace {
constexpr size_t kMaxBytes = 65536;
void Nonblocking(int fd) { fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK); }
}
Transport::Transport(int port) {
    if (port < 1 || port > 65535) throw std::runtime_error("调参端口必须为 1..65535");
    listener_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listener_ < 0) throw std::runtime_error("无法创建调参 socket");
    int yes = 1;
    setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) || listen(listener_, 2)) {
        close(listener_); listener_ = -1;
        throw std::runtime_error("无法监听调参端口 " + std::to_string(port) + "，检查端口是否被占用");
    }
    Nonblocking(listener_);
    thread_ = std::thread(&Transport::Run, this);
}
Transport::~Transport() {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    if (listener_ >= 0) close(listener_);
}
std::vector<Transport::Message> Transport::TakeRequests() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Message> result;
    // Bound work per physics step even if a client sends many requests.
    for (int i = 0; i < 8 && !requests_.empty(); ++i) {
        result.push_back(std::move(requests_.front())); requests_.pop_front();
    }
    return result;
}
void Transport::Reply(Message response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (replies_.size() < 64) replies_.push_back(std::move(response));
}
void Transport::Run() {
    int client = -1;
    uint64_t generation = 0;
    std::string input, output;
    auto disconnect = [&]() {
        if (client >= 0) close(client);
        client = -1; input.clear(); output.clear();
        std::lock_guard<std::mutex> lock(mutex_);
        requests_.clear(); replies_.clear();
    };
    while (!stop_) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            while (!replies_.empty()) {
                if (client >= 0 && replies_.front().connection == generation)
                    output += replies_.front().text + '\n';
                replies_.pop_front();
            }
        }
        if (output.size() > 4 * kMaxBytes) disconnect();
        pollfd fds[2]{{listener_, POLLIN, 0}, {client, short(POLLIN | (output.empty() ? 0 : POLLOUT)), 0}};
        if (poll(fds, 2, 20) < 0) { if (errno == EINTR) continue; break; }
        if (fds[0].revents & POLLIN) {
            int accepted = accept(listener_, nullptr, nullptr);
            if (accepted >= 0) {
                if (client >= 0) close(accepted); // One editor per simulator.
                else {
                    Nonblocking(accepted);
#ifdef SO_NOSIGPIPE
                    int yes = 1; setsockopt(accepted, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes));
#endif
                    client = accepted; ++generation;
                }
            }
        }
        if (client < 0) continue;
        if (fds[1].revents & (POLLERR | POLLHUP | POLLNVAL)) { disconnect(); continue; }
        if (fds[1].revents & POLLIN) {
            char buffer[8192];
            auto count = recv(client, buffer, sizeof(buffer), 0);
            if (count == 0 || (count < 0 && errno != EAGAIN && errno != EINTR)) { disconnect(); continue; }
            if (count > 0) input.append(buffer, static_cast<size_t>(count));
            if (input.size() > kMaxBytes) { disconnect(); continue; }
            size_t end;
            bool overflow = false;
            while ((end = input.find('\n')) != std::string::npos) {
                std::string line = input.substr(0, end); input.erase(0, end + 1);
                std::lock_guard<std::mutex> lock(mutex_);
                if (requests_.size() >= 64) { overflow = true; break; }
                requests_.push_back({generation, std::move(line)});
            }
            if (overflow) { disconnect(); continue; }
        }
        if ((fds[1].revents & POLLOUT) && !output.empty()) {
#ifdef MSG_NOSIGNAL
            constexpr int flags = MSG_NOSIGNAL;
#else
            constexpr int flags = 0;
#endif
            auto count = send(client, output.data(), output.size(), flags);
            if (count > 0) output.erase(0, static_cast<size_t>(count));
            else if (count < 0 && errno != EAGAIN && errno != EINTR) disconnect();
        }
    }
    disconnect();
}
}  // namespace sim_tuner
