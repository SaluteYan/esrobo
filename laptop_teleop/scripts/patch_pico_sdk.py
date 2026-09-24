"""Add atomic per-hand samples to the vendor SDK; applied before each build."""
from pathlib import Path
import sys


def patch(path):
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


if __name__ == '__main__':
    patch(Path(sys.argv[1]) / 'bindings/py_bindings.cpp')
