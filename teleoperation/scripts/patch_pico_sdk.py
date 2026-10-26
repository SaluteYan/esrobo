"""Add atomic component receipt snapshots to the vendor SDK before each build."""
from pathlib import Path
import re
import sys


def _patch_hand_v1(path):
    text = path.read_text()
    if '// ESROBO_HAND_SAMPLE_V1' in text:
        return
    # Fail closed on an upstream layout change, rather than applying half a patch.
    for side in ('Left', 'Right'):
        anchor = f'std::array<std::array<double, 7>, 26> {side}HandTrackingState;'
        if text.count(anchor) != 1:
            raise RuntimeError('Unsupported PICO SDK globals')
        text = text.replace(anchor, anchor + f'''
uint64_t {side}HandSequence = 0;
int64_t {side}HandTimestamp = 0;
std::chrono::steady_clock::time_point {side}HandReceived;
std::array<uint64_t, 26> {side}HandFlags{{}};''')
        line = f'{side}HandTrackingState[i] = stringToPoseArray({side.lower()}Hand["HandJointLocations"][i]["p"].get<std::string>());\n                        }}'
        if text.count(line) != 1:
            raise RuntimeError('Unsupported PICO SDK Hand parser')
        text = text.replace(line, line + f'''
                        for (int i = 0; i < 26; ++i) {{
                            {side}HandFlags[i] = static_cast<uint64_t>({side.lower()}Hand["HandJointLocations"][i].value("s", 0.0));
                        }}
                        {side}HandTimestamp = {side.lower()}Hand.value("timeStampNs", int64_t(0));
                        {side}HandReceived = std::chrono::steady_clock::now();
                        ++{side}HandSequence;''')
    anchor = '    m.def("get_left_hand_tracking_state",'
    if text.count(anchor) != 1:
        raise RuntimeError('Unsupported PICO SDK bindings')
    bindings = '    // ESROBO_HAND_SAMPLE_V1\n'
    for side in ('Left', 'Right'):
        bindings += f'''    m.def("get_{side.lower()}_hand_sample", []() {{
        std::lock_guard<std::mutex> lock({side.lower()}HandMutex);
        pybind11::dict sample;
        sample["sequence"] = {side}HandSequence;
        sample["active"] = {side}HandIsActive;
        sample["poses"] = {side}HandTrackingState;
        sample["flags"] = {side}HandFlags;
        sample["timestamp_ns"] = {side}HandTimestamp;
        sample["age_s"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - {side}HandReceived).count();
        return sample;
    }});
'''
    path.write_text(text.replace(anchor, bindings + anchor))


def patch(path):
    # Upgrade existing V1 installations as well as fresh vendor checkouts.
    _patch_hand_v1(path)
    text = path.read_text()
    if '// ESROBO_COMPONENT_SAMPLE_V2' not in text:
        anchor = '#include "PXREARobotSDK.h"'
        if text.count(anchor) != 1:
            raise RuntimeError('Unsupported PICO SDK includes')
        text = text.replace(anchor, anchor + '''
#include "pico_component_sample.hpp"
// ESROBO_COMPONENT_SAMPLE_V2
static esrobo_pico::Tracking ComponentTracking;

template<size_t N>
pybind11::dict component_sample(const esrobo_pico::Sample<N>& state) {
    pybind11::dict sample;
    sample["age_s"] = std::chrono::duration<double>(esrobo_pico::Clock::now() - state.received).count();
    sample["sequence"] = state.sequence;
    sample["active"] = state.active;
    sample["valid"] = state.sequence > 0;
    sample["poses"] = state.poses;
    if constexpr (N == 26) {
        sample["flags"] = state.flags;
        sample["timestamp_ns"] = state.timestamp_ns;
    }
    sample["packet_timestamp_ns"] = state.packet_timestamp_ns;
    sample["freshness_source"] = "component-packet";
    return sample;
}
''')
        anchor = '            json data = json::parse(dsj.stateJson);'
        if text.count(anchor) != 1:
            raise RuntimeError('Unsupported PICO SDK callback')
        text = text.replace(anchor, '            const auto received = esrobo_pico::Clock::now();\n' + anchor)
        anchor = '                auto value = json::parse(data["value"].get<std::string>());'
        if text.count(anchor) != 1:
            raise RuntimeError('Unsupported PICO SDK value parser')
        text = text.replace(anchor, anchor + '\n                ComponentTracking.ingest(value, received);')
        anchor = 'void init() {'
        if text.count(anchor) != 1:
            raise RuntimeError('Unsupported PICO SDK initialization')
        text = text.replace(anchor, anchor + '\n    ComponentTracking.reset();')
        for side in ('left', 'right'):
            start = text.index(f'    m.def("get_{side}_hand_sample",')
            end = text.index('    });', start) + len('    });')
            text = text[:start] + f'''    m.def("get_{side}_hand_sample", []() {{
        return component_sample(ComponentTracking.{side}.read());
    }});''' + text[end:]
        anchor = '    // Body tracking functions'
        if text.count(anchor) != 1:
            raise RuntimeError('Unsupported PICO SDK Body bindings')
        text = text.replace(anchor, '''    m.def("get_body_sample", []() {
        return component_sample(ComponentTracking.body.read());
    });
''' + anchor)
    if '// ESROBO_BODY_TIMESTAMP_CLEANUP_V3' not in text:
        def remove(pattern, *, count=1):
            nonlocal text
            text, replaced = re.subn(pattern, '', text, flags=re.MULTILINE)
            if replaced != count:
                raise RuntimeError('Unsupported PICO SDK Body timestamp layout')
        remove(r'^std::array<int64_t, 24> BodyJointsTimestamp;[^\n]*\n')
        remove(r'^int64_t BodyTimeStampNs = 0;[^\n]*\n')
        remove(r'\s+if \(body.contains\("timeStampNs"\)\) \{\s+BodyTimeStampNs = body\["timeStampNs"\].get<int64_t>\(\);\s+\}')
        remove(r'\s+// Parse IMU timestamp\s+if \(joint.contains\("t"\)\) \{\s+BodyJointsTimestamp\[i\] = joint\["t"\].get<int64_t>\(\);\s+\}')
        remove(r'std::array<int64_t, 24> getBodyJointsTimestamp\(\) \{[^}]*\}\n')
        remove(r'int64_t getBodyTimeStampNs\(\) \{[^}]*\}\n')
        remove(r'^    m.def\("get_body_joints_timestamp"[^\n]*\n')
        remove(r'^    m.def\("get_body_timestamp_ns"[^\n]*\n')
        # Existing V2 builds exposed obsolete Body fields. Fresh builds above
        # already expose only the Hand-specific optional stamp and flags.
        if '    sample["joint_timestamps"] = state.joint_timestamps;' in text:
            remove(r'^    sample\["joint_timestamps"\][^\n]*\n')
            text = text.replace('    sample["flags"] = state.flags;\n', '')
            text = text.replace('    sample["timestamp_ns"] = state.timestamp_ns;', '''    if constexpr (N == 26) {
        sample["flags"] = state.flags;
        sample["timestamp_ns"] = state.timestamp_ns;
    }''')
        text = '// ESROBO_BODY_TIMESTAMP_CLEANUP_V3\n' + text
    if '// ESROBO_TRACKING_NOTIFICATIONS_V4' not in text:
        anchor = '    sample["sequence"] = state.sequence;'
        if text.count(anchor) != 1:
            raise RuntimeError('Unsupported PICO SDK component snapshot layout')
        text = text.replace(anchor, anchor + '\n    sample["received_monotonic_ns"] = state.received_monotonic_ns;')
        anchor = '    // Body tracking functions'
        if text.count(anchor) != 1:
            raise RuntimeError('Unsupported PICO SDK notification binding layout')
        text = text.replace(anchor, '''    // ESROBO_TRACKING_NOTIFICATIONS_V4
    m.def("get_tracking_updates_version", []() { return 1; });
    m.def("wait_tracking_updates", [](uint64_t after_id, int timeout_ms) {
        esrobo_pico::Tracking::Updates updates;
        {
            pybind11::gil_scoped_release release;
            updates = ComponentTracking.wait(after_id, timeout_ms);
        }
        pybind11::dict result;
        result["notification_id"] = updates.id;
        result["body"] = component_sample(updates.body);
        result["left"] = component_sample(updates.left);
        result["right"] = component_sample(updates.right);
        return result;
    }, pybind11::arg("after_id"), pybind11::arg("timeout_ms") = 50);
    m.def("get_tracking_diagnostics", []() {
        pybind11::dict result;
        auto add = [&](const char* name, const esrobo_pico::Counts& counts) {
            pybind11::dict item;
            item["committed"] = counts.committed;
            item["rejected"] = counts.rejected;
            item["invalid"] = counts.invalid;
            item["receipts_ns"] = std::vector<int64_t>(counts.receipts.begin(), counts.receipts.end());
            result[name] = item;
        };
        add("body", ComponentTracking.body.diagnostics());
        add("left", ComponentTracking.left.diagnostics());
        add("right", ComponentTracking.right.diagnostics());
        add("notification", ComponentTracking.diagnostics());
        return result;
    });
''' + anchor)
    if '// ESROBO_CALLBACK_DIAGNOSTICS_V5' not in text:
        start = text.index('void OnPXREAClientCallback(')
        end = text.index('\nvoid init()', start)
        callback = text[start:end]
        callback, count = re.subn(r'^\s*std::cout <<[^\n]*\n', '', callback, flags=re.MULTILINE)
        if count != 5:
            raise RuntimeError('Unsupported PICO SDK callback output layout')
        anchor = '        } catch (const json::exception& e) {\n            std::cerr << "JSON parsing error: " << e.what() << std::endl;'
        if callback.count(anchor) != 1:
            raise RuntimeError('Unsupported PICO SDK parser diagnostics layout')
        callback = callback.replace(anchor, '''        } catch (const std::exception&) {
            // ESROBO_CALLBACK_DIAGNOSTICS_V5: bounded native counter, no I/O.
            ComponentTracking.parser_error();''')
        text = text[:start] + callback + text[end:]
    path.write_text(text)
    header = Path(__file__).with_name('pico_component_sample.hpp')
    path.with_name(header.name).write_text(header.read_text())


if __name__ == '__main__':
    patch(Path(sys.argv[1]) / 'bindings/py_bindings.cpp')
