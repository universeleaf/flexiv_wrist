from __future__ import annotations

import tomllib
from copy import deepcopy
from pathlib import Path

import pytest

from controllers.config_io import (
    extract_toml_comments,
    normalize_config_document,
    rewrite_toml_document,
)


EXAMPLE = """# heading
[robot]
# robot speed
speed = 1.0
enabled = true
name = "rizon"
axes = [1.0, 0.0,
        -1.0]

[runtime]
arm = true
"""


def test_rewrite_preserves_comments_and_updates_multiline_array() -> None:
    template = tomllib.loads(EXAMPLE)
    candidate = {
        "robot": {
            "speed": 2,
            "enabled": False,
            "name": "Rizon 4s",
            "axes": [0, 1, -1],
        },
        "runtime": {"arm": True},
    }
    normalized = normalize_config_document(candidate, template)
    rewritten = rewrite_toml_document(EXAMPLE, normalized)
    parsed = tomllib.loads(rewritten)
    assert parsed == normalized
    assert "# robot speed" in rewritten
    assert parsed["robot"]["axes"] == [0.0, 1.0, -1.0]


def test_config_shape_rejects_missing_or_wrong_typed_fields() -> None:
    template = tomllib.loads(EXAMPLE)
    missing = {"robot": dict(template["robot"]), "runtime": {}}
    with pytest.raises(ValueError, match="fields differ"):
        normalize_config_document(missing, template)
    wrong = {section: dict(values) for section, values in template.items()}
    wrong["robot"]["enabled"] = "true"
    with pytest.raises(ValueError, match="must be a boolean"):
        normalize_config_document(wrong, template)


def test_extract_comments_routes_description_to_field() -> None:
    comments = extract_toml_comments(EXAMPLE)
    assert comments["robot.speed"] == "robot speed"


def test_ui_can_change_mode3_force_target_from_default_to_minus_five() -> None:
    path = Path(__file__).parents[2] / "config" / "nine_dof_teleop.toml"
    original = path.read_text(encoding="utf-8")
    template = tomllib.loads(original)
    candidate = deepcopy(template)
    candidate["pivot_orientation_osc"]["target_sensed_force_tool_z_n"] = -5.0
    normalized = normalize_config_document(candidate, template)
    rewritten = rewrite_toml_document(original, normalized)
    assert (
        tomllib.loads(rewritten)["pivot_orientation_osc"]
        ["target_sensed_force_tool_z_n"]
        == -5.0
    )
