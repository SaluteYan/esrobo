#!/usr/bin/env python3
"""Manage the vendor hand console on the robot; never send motion targets."""

import argparse
import hashlib
import http.client
import json
import re
import shlex
import subprocess
import sys
import time
import uuid
from pathlib import Path
from urllib.parse import urlsplit


DEFAULT_CONFIG = Path(__file__).with_name("config.json")
REMOTE_PROBE = r'''
import json, pathlib, subprocess
conflicts = []
for p in pathlib.Path('/proc').iterdir():
    if not p.name.isdigit():
        continue
    try:
        a = [s.decode(errors='replace') for s in (p/'cmdline').read_bytes().split(b'\0') if s]
    except OSError:
        continue
    if not a:
        continue
    first = [pathlib.Path(s).name for s in a[:3]]
    # Also reject legacy processes on robots that have not migrated yet.
    modules = ('esrobo_link.gateway', 'esrobo_teleop.teleop_node', 'esrobo_teleop.hand_teleop')
    if (any(x in first for x in ('linker_hand_sdk', 'linker_hand_sdk.py', 'hand_ros_bridge.py',
                                'dashboard', 'can-bridge'))
            or any(a[i:i+2] == ['-m', m] for i in range(len(a)-1) for m in modules)):
        conflicts.append({'pid': int(p.name), 'args': a})
interfaces = json.loads(subprocess.check_output(['ip', '-j', '-details', 'link', 'show', 'type', 'can']))
print(json.dumps({'conflicts': conflicts, 'interfaces': interfaces}))
'''


def load_config(path):
    c = json.loads(Path(path).read_text())
    if not re.fullmatch(r"[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+", c["ssh_target"]):
        raise ValueError("ssh_target 需要是 user@IPv4或主机名")
    remote = c["remote_dir"]
    if (not re.fullmatch(r"/home/[A-Za-z0-9_./-]+", remote)
            or ".." in Path(remote).parts or len(Path(remote).parts) < 4):
        raise ValueError("remote_dir 必须是 /home 下独立的绝对目录")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]+", c["service_name"]):
        raise ValueError("无效服务名")
    if type(c["console_port"]) is not int or not 1024 <= c["console_port"] <= 65535:
        raise ValueError("console_port 需要在 1024..65535 范围")
    if not re.fullmatch(r"[a-f0-9]{64}", c["binary_sha256"]):
        raise ValueError("binary_sha256 必须是 SHA256")
    # This deployment has fixed USB mappings. Reject arm/base/waist interfaces.
    if c["hand_interfaces"] != {"left": "can_hand2", "right": "can_hand1"}:
        raise ValueError("此脚本只允许已核实的左 can_hand2、右 can_hand1")
    u = urlsplit(c["local_dashboard_url"])
    if u.scheme != "http" or u.hostname not in ("127.0.0.1", "localhost") or u.path:
        raise ValueError("local_dashboard_url 必须指向本机 HTTP 控制台")
    return c


def ssh(c, args, *, interactive=False, accepted_codes=(0,)):
    flags = ["-t"] if interactive else ["-o", "BatchMode=yes"]
    r = subprocess.run(["ssh", *flags, "-o", "ConnectTimeout=5", c["ssh_target"],
                        shlex.join(args)], check=False, text=True,
                       stdout=None if interactive else subprocess.PIPE)
    if r.returncode not in accepted_codes:
        r.check_returncode()
    return "" if interactive else r.stdout.strip()


def service_state(c):
    output = ssh(c, ["systemctl", "show", c["service_name"] + ".service",
                     "-p", "LoadState", "-p", "ActiveState", "-p", "SubState", "-p", "Environment"],
                 accepted_codes=(0, 1, 4))
    state = dict(line.split("=", 1) for line in output.splitlines() if "=" in line)
    if "ActiveState" not in state:
        raise RuntimeError("无法读取机器人上的 systemd 服务状态")
    return state


def get_json(url):
    u = urlsplit(url)
    con = http.client.HTTPConnection(u.hostname, u.port, timeout=12)
    try:
        con.request("GET", u.path or "/")
        r = con.getresponse()
        if r.status != 200:
            raise RuntimeError(f"HTTP {r.status}: {url}")
        return json.load(r)
    finally:
        con.close()


def console_url(c):
    return f"http://{c['ssh_target'].split('@')[1]}:{c['console_port']}"


def ensure_handover_safe(s):
    if not s.get("connected") or s.get("owner"):
        raise RuntimeError("请先连接项目控制台 SSH，并释放正在操作的页面")
    if s.get("processes", {}).get("teleop", {}).get("running"):
        raise RuntimeError("请先通过项目控制台结束遥操作计算")
    robot = s.get("robot") or {}
    if robot.get("mode") != "IDLE" or not 0 <= robot.get("age_s", 999) < 1:
        raise RuntimeError("交接要求新鲜 IDLE 状态；按项目流程处理停止、支撑和恢复")
    feedback = robot.get("feedback") or {}
    members = feedback.get("sides") or {"single": feedback}
    if not all(item.get("enable_states") == [False] * 7 for item in members.values()):
        raise RuntimeError("机械臂七轴失能尚未得到反馈确认，取消交接")


def handover(c):
    """Use the project's existing guarded endpoints, with stricter preconditions."""
    base = c["local_dashboard_url"]
    names = ("gateway", "hand_bridge", "hands")
    s = get_json(base + "/api/state")
    if not any("esrobo_" + n in s.get("remote", {}).get("sessions", []) for n in names):
        print("项目网关、双手桥和双手驱动已经停止")
        return
    ensure_handover_safe(s)
    if "esrobo_gateway" not in s["remote"]["sessions"]:
        raise RuntimeError("没有可核验的网关会话，请先按项目流程交接")
    u = urlsplit(base)
    session = "hand-console-" + uuid.uuid4().hex
    for name in names:
        if "esrobo_" + name not in s["remote"]["sessions"]:
            continue
        # Recheck before requesting q; q can apply electronic stop on exit.
        if name == "gateway":
            ensure_handover_safe(get_json(base + "/api/state"))
        con = http.client.HTTPConnection(u.hostname, u.port, timeout=15)
        try:
            con.request("POST", "/api/action", json.dumps({"action": "remote_stop",
                        "name": name, "session": session}),
                        {"Origin": base, "Content-Type": "application/json", "X-Head-Control": "1"})
            r = con.getresponse()
            result = json.load(r)
            if r.status != 200 or not result.get("ok"):
                raise RuntimeError(f"停止 {name} 失败：{result}")
        finally:
            con.close()
        end = time.monotonic() + 12
        while True:
            s = get_json(base + "/api/state")
            if "esrobo_" + name not in s["remote"]["sessions"]:
                break
            if time.monotonic() > end:
                raise RuntimeError(f"等待 {name} 退出超时，取消后续操作")
            time.sleep(.3)
        print("已停止：", name)


def deploy(c):
    binary = Path(c["local_binary"]).expanduser()
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    if digest != c["binary_sha256"]:
        raise RuntimeError("本地程序校验不符，取消部署")
    target = c["remote_dir"] + "/dashboard"
    check = "if test -e {0}; then sha256sum {0}; fi".format(shlex.quote(target))
    existing = ssh(c, ["sh", "-c", check])
    if existing:
        if existing.split()[0] != digest:
            raise RuntimeError("机器人已有不同程序，取消覆盖；先核对版本和服务状态")
        print("机器人程序已存在，SHA256 一致")
        return
    if service_state(c).get("ActiveState") not in ("inactive", "failed"):
        raise RuntimeError("请先停止厂商控制台服务，再部署程序")
    ssh(c, ["mkdir", "-p", c["remote_dir"]])
    # Upload to a temporary name, verify, then move to the executable path.
    upload = target + ".upload-" + uuid.uuid4().hex
    subprocess.run(["scp", "-q", str(binary), c["ssh_target"] + ":" + upload], check=True)
    if ssh(c, ["sha256sum", upload]).split()[0] != digest:
        raise RuntimeError("远端文件校验失败，未替换 dashboard")
    ssh(c, ["chmod", "755", upload])
    ssh(c, ["mv", "--", upload, target])
    print("已部署：", c["ssh_target"] + ":" + target)


def status(c, device_data=None):
    state = service_state(c)
    print("服务：", state.get("ActiveState"), state.get("SubState"))
    print("接口配置：", state.get("Environment", ""))
    print("网页：", console_url(c))
    if state.get("ActiveState") == "active":
        data = device_data if device_data is not None else get_json(console_url(c) + "/api/hand/devices")
        if data.get("status") != "ok":
            raise RuntimeError(f"设备查询失败：{data}")
        for item in data.get("data", []):
            if item.get("interface") in c["hand_interfaces"].values():
                print(json.dumps(item, ensure_ascii=False))


def start(c, side):
    selected = [c["hand_interfaces"][k] for k in ("left", "right") if side in (k, "both")]
    env = "CAN_PORTS=" + ",".join(selected)
    state = service_state(c)
    if state.get("ActiveState") == "active":
        if env not in shlex.split(state.get("Environment", "")):
            raise RuntimeError("已有控制台使用不同接口；先 stop，再以目标侧 start")
        status(c)
        return
    if state.get("ActiveState") not in ("inactive", "failed"):
        raise RuntimeError("服务正在切换状态，请稍后重新检查")
    probe = json.loads(ssh(c, ["python3", "-c", REMOTE_PROBE]))
    if probe["conflicts"]:
        raise RuntimeError("仍有冲突进程，未启动：" + json.dumps(probe["conflicts"], ensure_ascii=False)
                           + "；先运行 handover，或按项目停止流程处理")
    present = {item["ifname"] for item in probe["interfaces"]}
    if not set(selected) <= present:
        raise RuntimeError(f"缺少接口 {set(selected) - present}；请先核对机器人 USB/CAN 映射")
    binary = c["remote_dir"] + "/dashboard"
    if ssh(c, ["sha256sum", binary]).split()[0] != c["binary_sha256"]:
        raise RuntimeError("机器人程序校验失败，取消启动")
    if state.get("ActiveState") == "failed":
        ssh(c, ["sudo", "systemctl", "reset-failed", c["service_name"]], interactive=True)
    ssh(c, ["sudo", "systemd-run", "--unit=" + c["service_name"],
            "--description=Linker Hand Console", "--working-directory=" + c["remote_dir"],
            "--setenv=" + env, binary, "--port", str(c["console_port"])], interactive=True)
    for _ in range(15):
        time.sleep(1)
        state = service_state(c)
        if state.get("ActiveState") == "failed":
            raise RuntimeError("控制台启动失败，请运行 logs 查看日志")
        try:
            if state.get("ActiveState") == "active":
                data = get_json(console_url(c) + "/api/hand/devices")
                if data.get("status") != "ok":
                    continue
                status(c, data)
                return
        except (OSError, RuntimeError):
            pass
    raise RuntimeError("等待控制台就绪超时，请运行 status 和 logs 检查")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    sub = p.add_subparsers(dest="action", required=True)
    for action in ("deploy", "handover", "status", "stop", "open", "logs"):
        sub.add_parser(action)
    sub.add_parser("start").add_argument("--side", choices=("left", "right", "both"), default="both")
    args = p.parse_args()
    try:
        c = load_config(args.config)
        if args.action == "deploy":
            deploy(c)
        elif args.action == "handover":
            handover(c)
        elif args.action == "start":
            start(c, args.side)
        elif args.action == "status":
            status(c)
        elif args.action == "stop":
            if service_state(c).get("LoadState") == "not-found":
                print("控制台服务已经停止")
                return
            ssh(c, ["sudo", "systemctl", "stop", c["service_name"]], interactive=True)
            state = service_state(c)
            if state.get("ActiveState") not in ("inactive", "failed"):
                raise RuntimeError("服务尚未确认停止")
            print("控制台已停止；它管理的手部 CAN 接口可能已关闭，恢复项目驱动前需检查")
        elif args.action == "logs":
            print(ssh(c, ["journalctl", "-u", c["service_name"], "-n", "60", "--no-pager"]))
        elif args.action == "open":
            subprocess.run(["xdg-open", console_url(c)], check=True)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        p.exit(1, f"错误：{exc}\n")


if __name__ == "__main__":
    main()
