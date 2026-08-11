"""Small TOML helpers used by the dependency-free local control UI."""

from __future__ import annotations

import json
import math
import re
from typing import Any, Mapping


_SECTION_RE = re.compile(r"^\s*\[([A-Za-z0-9_.-]+)\]\s*(?:#.*)?$")
_ASSIGNMENT_RE = re.compile(r"^(\s*)([A-Za-z0-9_-]+)(\s*=\s*)(.*)$")


def _normalize_value(candidate: Any, template: Any, path: str) -> Any:
    if isinstance(template, bool):
        if not isinstance(candidate, bool):
            raise ValueError(f"{path} must be a boolean")
        return candidate
    if isinstance(template, int):
        if isinstance(candidate, bool) or not isinstance(candidate, int):
            raise ValueError(f"{path} must be an integer")
        return candidate
    if isinstance(template, float):
        if isinstance(candidate, bool) or not isinstance(candidate, (int, float)):
            raise ValueError(f"{path} must be a number")
        value = float(candidate)
        if not math.isfinite(value):
            raise ValueError(f"{path} must be finite")
        return value
    if isinstance(template, str):
        if not isinstance(candidate, str):
            raise ValueError(f"{path} must be a string")
        return candidate
    if isinstance(template, list):
        if not isinstance(candidate, list):
            raise ValueError(f"{path} must be an array")
        if len(candidate) != len(template):
            raise ValueError(f"{path} must contain {len(template)} elements")
        return [
            _normalize_value(value, expected, f"{path}[{index}]")
            for index, (value, expected) in enumerate(zip(candidate, template))
        ]
    raise ValueError(f"{path} has unsupported UI type {type(template).__name__}")


def normalize_config_document(
    candidate: Mapping[str, Any], template: Mapping[str, Any]
) -> dict[str, Any]:
    """Validate UI JSON against the existing TOML structure and types."""
    if not isinstance(candidate, Mapping):
        raise ValueError("configuration must be an object")
    if set(candidate) != set(template):
        missing = sorted(set(template) - set(candidate))
        extra = sorted(set(candidate) - set(template))
        raise ValueError(f"configuration sections differ; missing={missing}, extra={extra}")
    normalized: dict[str, Any] = {}
    for section, expected_values in template.items():
        values = candidate[section]
        if not isinstance(expected_values, Mapping) or not isinstance(values, Mapping):
            raise ValueError(f"{section} must be a TOML section")
        if set(values) != set(expected_values):
            missing = sorted(set(expected_values) - set(values))
            extra = sorted(set(values) - set(expected_values))
            raise ValueError(
                f"{section} fields differ; missing={missing}, extra={extra}"
            )
        normalized[section] = {
            key: _normalize_value(values[key], expected, f"{section}.{key}")
            for key, expected in expected_values.items()
        }
    return normalized


def format_toml_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("the UI cannot write NaN/Inf to TOML")
        rendered = repr(value)
        return rendered if any(marker in rendered for marker in (".", "e", "E")) else rendered + ".0"
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, list):
        return "[" + ", ".join(format_toml_value(item) for item in value) + "]"
    raise ValueError(f"cannot write TOML value of type {type(value).__name__}")


def _bracket_depth(text: str) -> int:
    depth = 0
    quote = ""
    escaped = False
    for character in text:
        if escaped:
            escaped = False
            continue
        if quote:
            if character == "\\" and quote == '"':
                escaped = True
            elif character == quote:
                quote = ""
            continue
        if character in ('"', "'"):
            quote = character
        elif character == "#":
            break
        elif character in "[{":
            depth += 1
        elif character in "]}":
            depth -= 1
    return depth


def _inline_comment(value_text: str) -> str:
    quote = ""
    escaped = False
    for index, character in enumerate(value_text):
        if escaped:
            escaped = False
            continue
        if quote:
            if character == "\\" and quote == '"':
                escaped = True
            elif character == quote:
                quote = ""
            continue
        if character in ('"', "'"):
            quote = character
        elif character == "#":
            return value_text[index:].rstrip()
    return ""


def rewrite_toml_document(original: str, document: Mapping[str, Any]) -> str:
    """Replace existing assignment values while retaining comments and order."""
    lines = original.splitlines(keepends=True)
    result: list[str] = []
    section = ""
    seen: set[tuple[str, str]] = set()
    index = 0
    while index < len(lines):
        line = lines[index]
        section_match = _SECTION_RE.match(line.rstrip("\r\n"))
        if section_match:
            section = section_match.group(1)
            result.append(line)
            index += 1
            continue
        assignment = _ASSIGNMENT_RE.match(line.rstrip("\r\n"))
        if assignment and section in document:
            indent, key, separator, initial_value = assignment.groups()
            if key in document[section]:
                end = index
                depth = _bracket_depth(initial_value)
                while depth > 0 and end + 1 < len(lines):
                    end += 1
                    depth += _bracket_depth(lines[end])
                comment = _inline_comment(initial_value) if end == index else ""
                replacement = (
                    f"{indent}{key}{separator}{format_toml_value(document[section][key])}"
                    + (f" {comment}" if comment else "")
                    + "\n"
                )
                result.append(replacement)
                seen.add((section, key))
                index = end + 1
                continue
        result.append(line)
        index += 1

    expected = {
        (section_name, key)
        for section_name, values in document.items()
        for key in values
    }
    if seen != expected:
        missing = sorted(".".join(path) for path in expected - seen)
        raise ValueError(f"fields not found in the original TOML: {missing}")
    return "".join(result)


def extract_toml_comments(original: str) -> dict[str, str]:
    """Return comment blocks immediately preceding configuration fields."""
    comments: dict[str, str] = {}
    section = ""
    pending: list[str] = []
    for line in original.splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            pending.append(stripped[1:].strip())
            continue
        section_match = _SECTION_RE.match(line)
        if section_match:
            section = section_match.group(1)
            pending.clear()
            continue
        assignment = _ASSIGNMENT_RE.match(line)
        if assignment and section:
            key = assignment.group(2)
            if pending:
                comments[f"{section}.{key}"] = " ".join(pending)
            pending.clear()
            continue
        if not stripped:
            pending.clear()
    return comments
