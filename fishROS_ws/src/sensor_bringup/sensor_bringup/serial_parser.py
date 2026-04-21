"""Helpers for turning line-oriented serial JSON into message fields."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ParsedSerialLine:
    raw_line: str
    raw_json: str = ''
    numeric_keys: list[str] = field(default_factory=list)
    numeric_values: list[float] = field(default_factory=list)
    string_keys: list[str] = field(default_factory=list)
    string_values: list[str] = field(default_factory=list)
    parse_ok: bool = False
    error: str = ''


def _stringify_value(value: Any) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value is None:
        return 'null'
    if isinstance(value, str):
        return value
    return json.dumps(value, sort_keys=True)


def parse_serial_json_line(raw_line: str) -> ParsedSerialLine:
    """Parse one decoded serial line as a JSON object.

    Numeric JSON values become parallel key/value arrays. Strings, bools, null,
    lists, and objects are preserved as string values so unknown future sensor
    fields are still visible in bags.
    """
    parsed = ParsedSerialLine(raw_line=raw_line)
    stripped = raw_line.strip()
    if not stripped:
        parsed.error = 'empty line'
        return parsed

    try:
        payload = json.loads(stripped)
    except json.JSONDecodeError as exc:
        parsed.error = f'json decode error: {exc.msg}'
        return parsed

    if not isinstance(payload, dict):
        parsed.error = 'json payload is not an object'
        return parsed

    parsed.parse_ok = True
    parsed.raw_json = json.dumps(
        payload,
        sort_keys=True,
        separators=(',', ':'),
    )

    for key, value in payload.items():
        key_text = str(key)
        if isinstance(value, bool):
            parsed.string_keys.append(key_text)
            parsed.string_values.append(_stringify_value(value))
        elif isinstance(value, (int, float)):
            parsed.numeric_keys.append(key_text)
            parsed.numeric_values.append(float(value))
        else:
            parsed.string_keys.append(key_text)
            parsed.string_values.append(_stringify_value(value))

    return parsed
