#!/usr/bin/env python3
"""
Convert toolpath/path.json into a COMPAS geometry JSON file.

Project structure:

SANDSKRIPT_COMPAS_RRC/
├── toolpath/
│   └── path.json
├── converted_toolpath/
│   └── path_compas.json
└── 303_convert_to_compas_json.py
"""

from __future__ import annotations

import json
import math
import uuid
from pathlib import Path
from typing import Any, Iterable


# The folder containing this Python file is treated as the project root.
PROJECT_ROOT = Path(__file__).resolve().parent

INPUT_PATH = PROJECT_ROOT / "toolpath" / "path.json"
OUTPUT_DIR = PROJECT_ROOT / "converted_toolpath"
OUTPUT_PATH = OUTPUT_DIR / "path_compas_1.json"

# Input coordinates are stored in metres.
# COMPAS RRC robot coordinates are normally used in millimetres.
COORDINATE_SCALE = 1000.0

DEFAULT_WOBJ_ORIGIN = [0.0, 0.0, 0.0]
DEFAULT_WOBJ_XAXIS = [1000.0, 0.0, 0.0]
DEFAULT_WOBJ_YAXIS = [0.0, 1000.0, 0.0]


def new_guid() -> str:
    """Create a unique GUID for a serialized COMPAS object."""
    return str(uuid.uuid4())


def scaled_vector(
    values: Iterable[Any],
    scale: float = 1.0,
) -> list[float]:
    """Validate a 3D vector and optionally scale it."""
    result = [float(value) * scale for value in values]

    if len(result) != 3:
        raise ValueError(f"Expected a 3D vector, received: {result!r}")

    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"Vector contains a non-finite value: {result!r}")

    # Replace -0.0 with 0.0 for cleaner JSON.
    return [0.0 if abs(value) < 1e-12 else value for value in result]


def make_compas_frame(
    point: Iterable[Any],
    xaxis: Iterable[Any],
    yaxis: Iterable[Any],
) -> dict[str, Any]:
    """Create a serialized COMPAS Frame."""
    return {
        "dtype": "compas.geometry/Frame",
        "data": {
            "point": scaled_vector(point, COORDINATE_SCALE),
            "xaxis": scaled_vector(xaxis),
            "yaxis": scaled_vector(yaxis),
        },
        "guid": new_guid(),
    }


def make_compas_point(point: Iterable[Any]) -> dict[str, Any]:
    """Create a serialized COMPAS Point."""
    return {
        "dtype": "compas.geometry/Point",
        "data": scaled_vector(point),
        "guid": new_guid(),
    }


def extract_frames(source: dict[str, Any]) -> list[dict[str, Any]]:
    """Flatten every point from every stroke into one COMPAS frame list."""
    strokes = source.get("strokes")

    if not isinstance(strokes, list):
        raise ValueError("Input JSON must contain a top-level 'strokes' list.")

    frames: list[dict[str, Any]] = []

    for stroke_index, stroke in enumerate(strokes):
        if not isinstance(stroke, list):
            raise ValueError(f"Stroke {stroke_index} is not a list.")

        for point_index, item in enumerate(stroke):
            try:
                plane = item["plane"]
                origin = plane["origin"]
                xaxis = plane["xaxis"]
                yaxis = plane["yaxis"]
            except (KeyError, TypeError) as error:
                raise ValueError(
                    f"Missing plane information at stroke {stroke_index}, "
                    f"point {point_index}."
                ) from error

            frames.append(
                make_compas_frame(
                    point=origin,
                    xaxis=xaxis,
                    yaxis=yaxis,
                )
            )

    return frames


def convert_to_compas(source: dict[str, Any]) -> dict[str, Any]:
    """Convert the source toolpath into the COMPAS JSON structure."""
    return {
        "frames": extract_frames(source),
        "wobj_origin": make_compas_point(DEFAULT_WOBJ_ORIGIN),
        "wobj_xaxis": make_compas_point(DEFAULT_WOBJ_XAXIS),
        "wobj_yaxis": make_compas_point(DEFAULT_WOBJ_YAXIS),
    }


def main() -> None:
    if not INPUT_PATH.is_file():
        raise FileNotFoundError(
            f"Could not find the input toolpath:\n{INPUT_PATH}"
        )

    with INPUT_PATH.open("r", encoding="utf-8") as input_file:
        source = json.load(input_file)

    converted = convert_to_compas(source)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    with OUTPUT_PATH.open("w", encoding="utf-8") as output_file:
        json.dump(
            converted,
            output_file,
            indent=4,
            ensure_ascii=False,
        )
        output_file.write("\n")

    print(f"Input:  {INPUT_PATH}")
    print(f"Output: {OUTPUT_PATH}")
    print(f"Frames converted: {len(converted['frames'])}")


if __name__ == "__main__":
    main()
