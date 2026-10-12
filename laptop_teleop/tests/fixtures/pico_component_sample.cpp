#include "pico_component_sample.hpp"
#include <cassert>
#include <thread>
#include <atomic>
using namespace esrobo_pico;

int main() {
    Tracking tracking;
    const auto t = Clock::now();
    json joints = json::array();
    for (int i = 0; i < 24; ++i) joints.push_back({{"p", "1,2,3,0,0,0,1"}, {"t", 0}});
    json body = {{"timeStampNs", 100}, {"Body", {{"joints", joints}}}};
    tracking.ingest(body, t);
    auto sample = tracking.body.read();
    assert(sample.sequence == 1 && sample.timestamp_ns == 0 && sample.packet_timestamp_ns == 100);
    assert(sample.received == t);
    tracking.ingest({{"timeStampNs", 101}, {"Head", {{"pose", "1,2,3,0,0,0,1"}}}}, t + std::chrono::seconds(1));
    assert(tracking.body.read().received == t); // Head cannot renew Body.
    tracking.ingest(body, t + std::chrono::seconds(2));
    body["timeStampNs"] = 99;
    tracking.ingest(body, t + std::chrono::seconds(2));
    assert(tracking.body.read().sequence == 1); // Duplicate and regressive packets.
    body["timeStampNs"] = 102;
    tracking.ingest(body, t + std::chrono::milliseconds(10));
    assert(tracking.body.read().sequence == 2); // Identical pose, real new packet.
    body["timeStampNs"] = 103;
    body["Body"]["joints"].erase(23);
    tracking.ingest(body, t + std::chrono::seconds(3));
    assert(tracking.body.read().sequence == 2); // Partial packet never partially commits.
    body["Body"]["joints"] = joints;
    for (const auto& invalid : {"1,2,3", "nan,2,3,0,0,0,1", "1,2,3,0,0,0,0", "1,2,3,0,0,0,1,9"}) {
        body["Body"]["joints"][21]["p"] = invalid;
        tracking.ingest(body, t + std::chrono::seconds(3));
        assert(tracking.body.read().sequence == 2);
    }
    json fingers = json::array();
    for (int i = 0; i < 26; ++i) fingers.push_back({{"p", "1,2,3,0,0,0,1"}, {"s", 15}});
    json hand = {{"timeStampNs", 110}, {"Hand", {{"rightHand", {
        {"isActive", 1}, {"HandJointLocations", fingers}, {"timeStampNs", 0}}}}}};
    tracking.ingest(hand, t + std::chrono::milliseconds(20));
    assert(tracking.right.read().sequence == 1 && tracking.left.read().sequence == 0);
    assert(tracking.body.read().sequence == 2); // Independent component ages.
    hand["timeStampNs"] = 111;
    tracking.ingest(hand, t + std::chrono::milliseconds(30));
    assert(tracking.right.read().sequence == 2); // Static hand accepted too.
    hand["timeStampNs"] = 112;
    hand["Hand"]["rightHand"]["timeStampNs"] = 10;
    tracking.ingest(hand, t + std::chrono::milliseconds(40));
    hand["timeStampNs"] = 113;
    tracking.ingest(hand, t + std::chrono::seconds(4));
    assert(tracking.right.read().sequence == 3); // Existing native stamp cannot be frozen.
    auto held = tracking.right.read();
    assert(held.received == t + std::chrono::milliseconds(40));
    hand["timeStampNs"] = 114;
    hand["Hand"]["rightHand"]["timeStampNs"] = 11;
    hand["Hand"]["rightHand"]["HandJointLocations"][0]["s"] = -1;
    tracking.ingest(hand, t + std::chrono::seconds(5));
    assert(tracking.right.read().sequence == 3);
    assert(tracking.body.read().received_monotonic_ns > 0);
    auto prior = tracking.wait(0, 0);
    auto id = prior.id;
    tracking.ingest({{"timeStampNs", 200}, {"Head", {}}});
    assert(tracking.wait(id, 1).id == id);
    std::atomic<bool> woke{false};
    std::thread waiter([&] { auto update = tracking.wait(id, 50); woke = update.id > id; });
    body["timeStampNs"] = 201;
    body["Body"]["joints"] = joints;
    tracking.ingest(body);
    waiter.join();
    assert(woke); // Commit before or after wait's check: no lost wakeup.
    auto counts = tracking.body.diagnostics();
    assert(counts.committed == 3 && counts.rejected == 2 && counts.invalid == 5);
    for (int i=202; i<500; ++i) { body["timeStampNs"] = i; tracking.ingest(body); }
    assert(tracking.body.diagnostics().receipts.size() == 200);
    tracking.reset();
    assert(tracking.body.read().sequence == 0 && tracking.right.read().sequence == 0);
}
