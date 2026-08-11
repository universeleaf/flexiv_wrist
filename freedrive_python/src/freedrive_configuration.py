"""Fixed Cartesian freedrive configuration for a Flexiv Rizon 4S."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

EXPECTED_ROBOT_MODEL = "Rizon4s"
EXPECTED_ROBOT_SOFTWARE_SERIES = "3.11"

CARTESIAN_MASK_PARAM = "floatingAxis"
ELBOW_PARAM = "enableElbowMotion"
CARTESIAN_MASK = (1, 1, 1, 1, 1, 1)
DISABLE_ELBOW_MOTION = 0


class ConfigurationError(ValueError):
    """Invalid local configuration."""


def normalize_software_version(version: str) -> str:
    text = str(version).strip().lower()
    return text[1:] if text.startswith("v") else text


def normalize_robot_model(model: str) -> str:
    """Normalize the model spelling used by different Robot Software builds."""
    return "".join(character for character in str(model).lower() if character.isalnum())


def software_version_is_supported(version: str) -> bool:
    """RDK 1.9.0 is compatible with Robot Software 3.11.x."""
    normalized = normalize_software_version(version)
    return normalized == EXPECTED_ROBOT_SOFTWARE_SERIES or normalized.startswith(
        f"{EXPECTED_ROBOT_SOFTWARE_SERIES}."
    )


@dataclass(frozen=True)
class FreedriveConfiguration:
    sample_period_s: float = 0.05
    debounce_s: float = 0.08
    startup_timeout_s: float = 30.0
    stop_timeout_s: float = 10.0
    diagnose_only: bool = False
    print_command_only: bool = False
    confirm_motion: bool = False

    def validate(self) -> None:
        for name, value in (
            ("sample_period_s", self.sample_period_s),
            ("startup_timeout_s", self.startup_timeout_s),
            ("stop_timeout_s", self.stop_timeout_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ConfigurationError(f"{name} must be a finite positive number")
        if not math.isfinite(self.debounce_s) or self.debounce_s < 0.0:
            raise ConfigurationError("debounce_s must be finite and >= 0")

    def primitive_name(self) -> str:
        return "FloatingCartesian"

    def primitive_params(self, coord_factory: Any | None = None) -> dict[str, Any]:
        """Return only the verified Cartesian floating and elbow parameters."""
        del coord_factory
        self.validate()
        return {
            CARTESIAN_MASK_PARAM: list(CARTESIAN_MASK),
            ELBOW_PARAM: DISABLE_ELBOW_MOTION,
        }

    def command_report(self, coord_factory: Any | None = None) -> dict[str, Any]:
        params = self.primitive_params(coord_factory)
        return {
            "execute_primitive": {
                "name": self.primitive_name(),
                "input_params": params,
                "control_mode": "NRT_PRIMITIVE_EXECUTION",
            },
            "effective_configuration": {
                "cartesian_directions": ["X", "Y", "Z", "Rx", "Ry", "Rz"],
                "floatingAxis": list(CARTESIAN_MASK),
                "enableElbowMotion": DISABLE_ELBOW_MOTION,
                "independent_joint_floating": False,
                "independent_elbow_null_space_motion": False,
                "other_parameters": "RDK/robot controller defaults",
            },
            "startup_sequence": {
                "starts_locked": True,
                "home_before_freedrive": False,
                "zero_ft_before_freedrive": True,
                "first_pedal_press": "start FloatingCartesian from the current pose",
                "second_pedal_press": "Stop() and return to locked/IDLE",
            },
        }
