"""Load and validate iKKEGOL foot-pedal YAML configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from src.freedrive_configuration import ConfigurationError

KNOWN_ACTIONS = frozenset(
    {
        "none",
        "freedrive_toggle",
        "portable_device",
        "pre_programmed",
        "teleop_7dof",
        "teleop_9dof",
        "teleop_pivot_orientation",
    }
)

IMPLEMENTED_ACTIONS = frozenset(
    {
        "none",
        "freedrive_toggle",
        "teleop_7dof",
        "teleop_9dof",
        "teleop_pivot_orientation",
    }
)

DEFAULT_FOOT_PEDAL_CONFIG = (
    Path(__file__).resolve().parents[1] / "config" / "foot_pedal.yaml"
)


def _parse_optional_int(value: Any, field: str) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise ConfigurationError(f"{field} must be an integer or null, not bool")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        text = value.strip().lower()
        if not text or text in {"null", "none"}:
            return None
        try:
            return int(text, 0)
        except ValueError as exc:
            raise ConfigurationError(f"{field} must be an integer (got {value!r})") from exc
    raise ConfigurationError(f"{field} must be an integer or null (got {type(value).__name__})")


def _parse_optional_str(value: Any, field: str) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


@dataclass(frozen=True)
class PedalBinding:
    """One physical pedal binding and its logical action."""

    pedal_id: str
    key_code: int | None
    action: str
    intended_function: str | None = None

    def is_armed(self) -> bool:
        """True when a key code is configured so the pedal can emit events."""
        return self.key_code is not None


@dataclass(frozen=True)
class FootPedalDeviceConfig:
    name_contains: str | None = "iKKEGOL"
    path: str | None = None
    vendor_id: int | None = None
    product_id: int | None = None


@dataclass(frozen=True)
class FootPedalConfiguration:
    """Foot-pedal settings loaded from YAML."""

    enabled: bool = True
    debounce_s: float = 0.08
    device: FootPedalDeviceConfig = FootPedalDeviceConfig()
    pedals: Mapping[str, PedalBinding] | None = None

    def __post_init__(self) -> None:
        if self.pedals is None:
            object.__setattr__(
                self,
                "pedals",
                {
                    "pedal_1": PedalBinding(
                        "pedal_1", None, "none", intended_function="portable_device"
                    ),
                    "pedal_2": PedalBinding("pedal_2", None, "freedrive_toggle"),
                    "pedal_3": PedalBinding(
                        "pedal_3", None, "none", intended_function="pre_programmed"
                    ),
                },
            )

    def validate(self) -> None:
        if self.debounce_s < 0.0:
            raise ConfigurationError("foot_pedal.debounce_ms/debounce_s must be >= 0")
        if not self.pedals:
            raise ConfigurationError("foot_pedal.pedals must define at least pedal_2")
        for pedal_id, binding in self.pedals.items():
            if binding.action not in KNOWN_ACTIONS:
                raise ConfigurationError(
                    f"{pedal_id}.action {binding.action!r} is unknown; "
                    f"expected one of {sorted(KNOWN_ACTIONS)}"
                )
            if binding.key_code is not None and binding.key_code < 0:
                raise ConfigurationError(f"{pedal_id}.key_code must be >= 0")
            if (
                binding.action == "freedrive_toggle"
                and pedal_id != "pedal_2"
                and binding.key_code is not None
            ):
                raise ConfigurationError(
                    "freedrive_toggle is only allowed on pedal_2 in this project"
                )

        codes = [
            b.key_code
            for b in self.pedals.values()
            if b.key_code is not None
        ]
        if len(codes) != len(set(codes)):
            raise ConfigurationError("pedal key_code values must be unique when set")

        if self.device.path is not None:
            path = Path(self.device.path)
            if "event" in path.name and path.parent == Path("/dev/input"):
                # Soft warning via exception only if the path looks like a volatile node
                # and no other matcher is present — still allow it, but prefer by-id.
                pass

    def pedal(self, pedal_id: str) -> PedalBinding:
        if not self.pedals or pedal_id not in self.pedals:
            raise ConfigurationError(f"unknown pedal id: {pedal_id}")
        return self.pedals[pedal_id]

    def freedrive_toggle_ready(self) -> bool:
        """True when Pedal 2 can drive Freedrive enter/exit toggles."""
        if not self.enabled:
            return False
        try:
            pedal = self.pedal("pedal_2")
        except ConfigurationError:
            return False
        return (
            pedal.action == "freedrive_toggle"
            and pedal.key_code is not None
        )

    def key_code_to_binding(self) -> dict[int, PedalBinding]:
        mapping: dict[int, PedalBinding] = {}
        if not self.pedals:
            return mapping
        for binding in self.pedals.values():
            if binding.key_code is not None:
                mapping[binding.key_code] = binding
        return mapping


def load_foot_pedal_configuration(
    path: str | Path | None = None,
) -> FootPedalConfiguration:
    """Load YAML foot-pedal configuration from *path* (default project config)."""
    config_path = Path(path) if path is not None else DEFAULT_FOOT_PEDAL_CONFIG
    if not config_path.is_file():
        raise ConfigurationError(f"foot pedal config not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)

    if not isinstance(raw, dict) or "foot_pedal" not in raw:
        raise ConfigurationError(
            f"{config_path}: top-level key 'foot_pedal' is required"
        )
    section = raw["foot_pedal"]
    if not isinstance(section, dict):
        raise ConfigurationError(f"{config_path}: foot_pedal must be a mapping")

    enabled = bool(section.get("enabled", True))
    if "debounce_ms" in section:
        debounce_s = float(section["debounce_ms"]) / 1000.0
    elif "debounce_s" in section:
        debounce_s = float(section["debounce_s"])
    else:
        debounce_s = 0.08

    device_raw = section.get("device") or {}
    if not isinstance(device_raw, dict):
        raise ConfigurationError("foot_pedal.device must be a mapping")

    device = FootPedalDeviceConfig(
        name_contains=_parse_optional_str(
            device_raw.get("name_contains", "iKKEGOL"), "device.name_contains"
        ),
        path=_parse_optional_str(device_raw.get("path"), "device.path"),
        vendor_id=_parse_optional_int(device_raw.get("vendor_id"), "device.vendor_id"),
        product_id=_parse_optional_int(
            device_raw.get("product_id"), "device.product_id"
        ),
    )

    pedals_raw = section.get("pedals") or {}
    if not isinstance(pedals_raw, dict):
        raise ConfigurationError("foot_pedal.pedals must be a mapping")

    pedals: dict[str, PedalBinding] = {}
    for pedal_id, entry in pedals_raw.items():
        if not isinstance(entry, dict):
            raise ConfigurationError(f"foot_pedal.pedals.{pedal_id} must be a mapping")
        action = str(entry.get("action", "none")).strip()
        intended = _parse_optional_str(
            entry.get("intended_function"), f"{pedal_id}.intended_function"
        )
        pedals[str(pedal_id)] = PedalBinding(
            pedal_id=str(pedal_id),
            key_code=_parse_optional_int(entry.get("key_code"), f"{pedal_id}.key_code"),
            action=action,
            intended_function=intended,
        )

    # Ensure the three logical pedals exist even if omitted from YAML.
    defaults = {
        "pedal_1": PedalBinding(
            "pedal_1", None, "none", intended_function="portable_device"
        ),
        "pedal_2": PedalBinding("pedal_2", None, "freedrive_toggle"),
        "pedal_3": PedalBinding(
            "pedal_3", None, "none", intended_function="pre_programmed"
        ),
    }
    for pedal_id, default in defaults.items():
        pedals.setdefault(pedal_id, default)

    config = FootPedalConfiguration(
        enabled=enabled,
        debounce_s=debounce_s,
        device=device,
        pedals=pedals,
    )
    config.validate()
    return config
