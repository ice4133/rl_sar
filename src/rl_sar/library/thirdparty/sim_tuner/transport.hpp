// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sim_tuner {
// Newline-delimited JSON over loopback TCP. No MuJoCo calls on this thread.
class Transport {
public:
    struct Message { uint64_t connection; std::string text; };
    explicit Transport(int port);
    ~Transport();
    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;
    std::vector<Message> TakeRequests();
    void Reply(Message response);
private:
    void Run();
    int listener_ = -1;
    std::atomic<bool> stop_{false};
    std::thread thread_;
    std::mutex mutex_;
    std::deque<Message> requests_, replies_;
};
}  // namespace sim_tuner
