from dataclasses import dataclass, fields
from pathlib import Path

import yaml

from esrobo_teleop.config import load_config

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT.parent


@dataclass
class Settings:
    robot_host: str = "192.168.10.100"
    robot_port: int = 16000
    side: str = "both"
    with_hand: bool = True
    hand_only: bool = False
    key_file: str = "~/.config/esrobo/robot-link.key"
    teleop_config: str = "../../teleoperation/config/teleop_config.yaml"
    input_host: str = "127.0.0.1"
    input_port: int = 15050
    input_max_age_s: float = 0.1
    feedback_max_age_s: float = 0.15
    hand_feedback_max_age_s: float = 0.15
    network_margin_s: float = 0.02
    rate_hz: float = 50
    minimum_control_rate_hz: float = 40
    hand_only_rate_tolerance_hz: float = 5
    control_gap_timeout_s: float = 0.08

    def validate(self):
        if self.side not in ("left", "right", "both"):
            raise ValueError("side must be left, right or both")
        if type(self.with_hand) is not bool or type(self.hand_only) is not bool:
            raise ValueError("with_hand and hand_only must be boolean")
        if self.side == "both" and (not self.with_hand or self.hand_only):
            raise ValueError("both requires arm+hand mode")
        if self.hand_only and not self.with_hand:
            raise ValueError("hand_only requires with_hand")
        if self.input_host != "127.0.0.1":
            raise ValueError("sensor adapters must run on this laptop's loopback")
        for port in (self.robot_port, self.input_port):
            if type(port) is not int or not 1 <= port <= 65535:
                raise ValueError("invalid UDP port")
        if not (0 < self.input_max_age_s <= .1 and 0 < self.feedback_max_age_s <= .15
                and self.feedback_max_age_s <= self.hand_feedback_max_age_s <= .5
                and 0 <= self.network_margin_s < self.feedback_max_age_s
                and 1 <= self.minimum_control_rate_hz <= self.rate_hz <= 50
                and 1/self.minimum_control_rate_hz < self.control_gap_timeout_s <= self.input_max_age_s):
            raise ValueError("invalid freshness or rate limits")
        if self.hand_only and not (0 <= self.hand_only_rate_tolerance_hz <= 5
                                   and self.minimum_control_rate_hz-self.hand_only_rate_tolerance_hz >= 35):
            raise ValueError("hand-only sustained control rate must remain at least 35 Hz")

    @property
    def sides(self):
        return ("left", "right") if self.side == "both" else (self.side,)


def read_settings(path):
    path = Path(path).expanduser().resolve()
    data = yaml.safe_load(path.read_text()) or {}
    unknown = set(data) - {f.name for f in fields(Settings)}
    if unknown:
        raise ValueError(f"unknown laptop settings: {sorted(unknown)}")
    settings = Settings(**data)
    settings.validate()
    config_path = Path(settings.teleop_config).expanduser()
    if not config_path.is_absolute():
        config_path = path.parent / config_path
    settings.teleop_config = str(config_path.resolve())
    key = Path(settings.key_file).expanduser()
    settings.key_file = str(key if key.is_absolute() else path.parent / key)
    return settings


def read_robot_config(settings):
    path = Path(settings.teleop_config)
    cfg = load_config(str(path))
    model = Path(cfg.ik.urdf_path)
    cfg.ik.urdf_path = str((model if model.is_absolute() else path.parent.parent / model).resolve())
    if not Path(cfg.ik.urdf_path).is_file():
        raise ValueError(f"URDF missing: {cfg.ik.urdf_path}")
    if not (cfg.ik.partition_terminal_wrist_ik and cfg.ik.enable_elbow_tasks):
        raise ValueError("laptop requires partitioned position IK with elbow tasks")
    if cfg.hand.model.upper() != "L10":
        raise ValueError("robot_link requires L10 physical hand mapping")
    return cfg
