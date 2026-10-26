// Component snapshots for the PICO binding. No sensor/control requests here.
#pragma once
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <vector>
#include <ctime>
#include <sstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace esrobo_pico {
using Clock = std::chrono::steady_clock;
using json = nlohmann::json;

// CLOCK_MONOTONIC is also Python time.monotonic's clock on this Linux host.
inline int64_t receipt_ns(Clock::time_point received) {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const auto delay = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - received).count();
    return int64_t(ts.tv_sec) * 1000000000 + ts.tv_nsec - delay;
}

struct Counts {
    uint64_t committed = 0, rejected = 0, invalid = 0;
    std::deque<int64_t> receipts;
};

inline std::array<double, 7> pose(const json& value) {
    std::array<double, 7> result{};
    std::istringstream stream(value.get<std::string>());
    std::string token;
    for (double& number : result) {
        if (!std::getline(stream, token, ',')) throw std::runtime_error("incomplete pose");
        size_t end = 0;
        number = std::stod(token, &end);
        if (end != token.size() || !std::isfinite(number)) throw std::runtime_error("invalid pose");
    }
    if (std::getline(stream, token, ',')) throw std::runtime_error("extra pose element");
    double norm = 0;
    for (size_t i = 3; i < 7; ++i) norm += result[i] * result[i];
    if (norm < .25) throw std::runtime_error("invalid quaternion");
    return result;
}

template<size_t N> struct Sample {
    std::array<std::array<double, 7>, N> poses{};
    std::array<uint64_t, N> flags{};
    uint64_t sequence = 0;
    int64_t packet_timestamp_ns = 0;
    int64_t timestamp_ns = 0; // Optional component timestamp, not invented.
    int active = 0;
    Clock::time_point received{};
    int64_t received_monotonic_ns = 0;
};

template<size_t N> class Cache {
    mutable std::mutex mutex;
    Sample<N> state;
    Counts counts;
public:
    void reset() {
        std::lock_guard<std::mutex> lock(mutex);
        state = Sample<N>{};
        counts = Counts{};
    }
    Sample<N> read() const {
        std::lock_guard<std::mutex> lock(mutex);
        return state;
    }
    Counts diagnostics() const { std::lock_guard<std::mutex> lock(mutex); return counts; }
    void invalid() { std::lock_guard<std::mutex> lock(mutex); ++counts.invalid; }
    bool commit(Sample<N> next) {
        std::lock_guard<std::mutex> lock(mutex);
        // Same/regressive source packets and frozen native sensor stamps
        // cannot renew an earlier sample. Restart acquisition after a clock reset.
        if (next.packet_timestamp_ns <= state.packet_timestamp_ns) { ++counts.rejected; return false; }
        if (next.timestamp_ns > 0 && state.timestamp_ns > 0 &&
            next.timestamp_ns <= state.timestamp_ns) { ++counts.rejected; return false; }
        next.sequence = state.sequence + 1;
        state = next;
        ++counts.committed;
        counts.receipts.push_back(next.received_monotonic_ns);
        if (counts.receipts.size() > 200) counts.receipts.pop_front();
        return true;
    }
};

struct Tracking {
    Cache<24> body;
    Cache<26> left, right;

    struct Updates { uint64_t id; Sample<24> body; Sample<26> left, right; };
    std::mutex notification_mutex;
    std::condition_variable notification;
    uint64_t update_id = 0;
    Counts notification_counts;
    void reset() {
        std::lock_guard<std::mutex> lock(notification_mutex);
        body.reset(); left.reset(); right.reset(); update_id = 0; notification_counts = Counts{};
    }
    Counts diagnostics() { std::lock_guard<std::mutex> lock(notification_mutex); return notification_counts; }
    void parser_error() { std::lock_guard<std::mutex> lock(notification_mutex); ++notification_counts.invalid; }
    Updates wait(uint64_t after_id, int timeout_ms) {
        std::unique_lock<std::mutex> lock(notification_mutex);
        notification.wait_for(lock, std::chrono::milliseconds(std::max(0, std::min(timeout_ms, 50))),
                              [&] { return update_id > after_id; });
        return {update_id, body.read(), left.read(), right.read()};
    }

    void ingest(const json& value, Clock::time_point received = Clock::now()) {
        int64_t packet_stamp = 0;
        try { packet_stamp = value.value("timeStampNs", int64_t(0)); }
        catch (const std::exception&) { return; }
        // Current full-body protocol supplies this same-packet clock. A global
        // XR getter, or a polling clock, is never a substitute for it.
        if (packet_stamp <= 0) return;
        const auto monotonic_ns = receipt_ns(received);
        std::unique_lock<std::mutex> lock(notification_mutex);
        bool changed = false;
        try {
            if (value.contains("Body")) {
                const auto& component = value.at("Body");
                const auto& joints = component.at("joints");
                if (!joints.is_array() || joints.size() != 24) throw std::runtime_error("incomplete Body");
                Sample<24> next;
                for (size_t i = 0; i < 24; ++i) {
                    next.poses[i] = pose(joints[i].at("p"));
                }
                next.packet_timestamp_ns = packet_stamp;
                next.active = 1;
                next.received = received;
                next.received_monotonic_ns = monotonic_ns;
                changed = body.commit(next) || changed;
            }
        } catch (const std::exception&) { body.invalid(); } // Age still grows.
        auto ingest_hand = [&](const char* name, Cache<26>& cache) {
            try {
                if (!value.contains("Hand") || !value.at("Hand").is_object()) return;
                if (!value.at("Hand").contains(name)) return;
                const auto& component = value.at("Hand").at(name);
                const auto& joints = component.at("HandJointLocations");
                if (!joints.is_array() || joints.size() != 26) throw std::runtime_error("incomplete Hand");
                Sample<26> next;
                for (size_t i = 0; i < 26; ++i) {
                    next.poses[i] = pose(joints[i].at("p"));
                    const double flags = joints[i].value("s", 0.0);
                    if (!std::isfinite(flags) || flags < 0 || flags > 15 || std::floor(flags) != flags)
                        throw std::runtime_error("invalid Hand flags");
                    next.flags[i] = static_cast<uint64_t>(flags);
                }
                next.timestamp_ns = component.value("timeStampNs", int64_t(0));
                next.packet_timestamp_ns = packet_stamp;
                next.active = component.value("isActive", 0);
                next.received = received;
                next.received_monotonic_ns = monotonic_ns;
                changed = cache.commit(next) || changed;
            } catch (const std::exception&) { cache.invalid(); }
        };
        ingest_hand("leftHand", left);
        ingest_hand("rightHand", right);
        if (changed) {
            ++update_id;
            ++notification_counts.committed;
            notification_counts.receipts.push_back(monotonic_ns);
            if (notification_counts.receipts.size() > 200) notification_counts.receipts.pop_front();
        }
        lock.unlock();
        if (changed) notification.notify_all();
    }
};
} // namespace esrobo_pico
