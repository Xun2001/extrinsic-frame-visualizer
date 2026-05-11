#!/usr/bin/env python3
"""
Extrinsic visualizer for validating multi-sensor calibration matrices.

Convention
----------
The input matrix follows:

    p_target = T_target_source @ p_source

where:
    - T[:3, :3] is the rotation from the source frame to the target frame
    - T[:3, 3] is the source-frame origin expressed in the target frame

README-style examples
---------------------
1. Directly pass a matrix from the command line:

   python extrinsic_visualizer.py \
       --matrix "1 0 0 0.4; 0 0 -1 0.1; 0 1 0 0.2; 0 0 0 1" \
       --source LiDAR \
       --target IMU \
       --output lidar_to_imu.html

2. Pass a JSON-style matrix literal:

   python extrinsic_visualizer.py \
       --matrix "[[1, 0, 0, 0.4], [0, 0, -1, 0.1], [0, 1, 0, 0.2], [0, 0, 0, 1]]" \
       --source Camera \
       --target LiDAR

3. Load from a text file:

   python extrinsic_visualizer.py \
       --matrix-file extrinsic.txt \
       --source LiDAR \
       --target Camera

4. Load from a JSON file and invert before display:

   python extrinsic_visualizer.py \
       --matrix-file imu_to_lidar.json \
       --inverse \
       --source LiDAR \
       --target IMU
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np
import plotly.graph_objects as go


AXIS_COLORS = {"X": "#d62728", "Y": "#2ca02c", "Z": "#1f77b4"}
AXIS_VECTORS = {
    "X": np.array([1.0, 0.0, 0.0]),
    "Y": np.array([0.0, 1.0, 0.0]),
    "Z": np.array([0.0, 0.0, 1.0]),
}
NUMBER_PATTERN = re.compile(r"[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?")
MATRIX_KEYS = (
    "matrix",
    "T",
    "transform",
    "extrinsic",
    "T_target_source",
    "T_source_target",
)
USAGE_EXAMPLES = """Examples:
  python extrinsic_visualizer.py --matrix "1 0 0 0.4; 0 0 -1 0.1; 0 1 0 0.2; 0 0 0 1" --source LiDAR --target IMU
  python extrinsic_visualizer.py --matrix "[[1,0,0,0.4],[0,0,-1,0.1],[0,1,0,0.2],[0,0,0,1]]" --source Camera --target LiDAR
  python extrinsic_visualizer.py --matrix-file extrinsic.txt --source LiDAR --target Camera --output lidar_camera.html
  python extrinsic_visualizer.py --matrix-file imu_to_lidar.json --inverse --source LiDAR --target IMU
"""


def parse_matrix_text(text: str) -> np.ndarray:
    """Parse a 4x4 matrix from JSON-like or plain text input."""
    raw = text.strip()
    if not raw:
        raise ValueError("Matrix text is empty.")

    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        parsed = None

    if parsed is not None:
        matrix = np.asarray(parsed, dtype=float)
        if matrix.shape == (4, 4):
            return matrix
        if matrix.size == 16:
            return matrix.reshape(4, 4)
        raise ValueError(f"JSON matrix must be 4x4 or contain 16 values, got shape {matrix.shape}.")

    normalized = raw.replace(",", " ")
    normalized = normalized.replace("[", " ").replace("]", " ")
    row_chunks = [chunk.strip() for chunk in re.split(r"[;\n]+", normalized) if chunk.strip()]

    if row_chunks:
        rows = []
        for chunk in row_chunks:
            matches = NUMBER_PATTERN.findall(chunk)
            if not matches:
                continue
            rows.append(np.array([float(value) for value in matches], dtype=float))

        if len(rows) == 4 and all(row.size == 4 for row in rows):
            return np.vstack(rows)

    flat_values = np.array([float(value) for value in NUMBER_PATTERN.findall(normalized)], dtype=float)
    if flat_values.size == 16:
        return flat_values.reshape(4, 4)

    raise ValueError(
        "Unable to parse a 4x4 matrix. Supported formats include "
        "'a b c d; e f g h; ...' or JSON lists such as '[[...], ...]'."
    )


def load_matrix(matrix_arg: str | None, matrix_file: str | None) -> np.ndarray:
    """Load a 4x4 matrix from CLI text or a text/JSON file."""
    if bool(matrix_arg) == bool(matrix_file):
        raise ValueError("Specify exactly one of --matrix or --matrix-file.")

    if matrix_arg:
        return parse_matrix_text(matrix_arg)

    path = Path(matrix_file).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Matrix file not found: {path}")

    content = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        data = json.loads(content)
        if isinstance(data, dict):
            for key in MATRIX_KEYS:
                if key in data:
                    return parse_matrix_text(json.dumps(data[key]))
            for value in data.values():
                try:
                    return parse_matrix_text(json.dumps(value))
                except (TypeError, ValueError):
                    continue
            raise ValueError(f"No 4x4 matrix field found in JSON file: {path}")
        return parse_matrix_text(json.dumps(data))

    return parse_matrix_text(content)


def rotation_diagnostics(rotation: np.ndarray) -> tuple[float, float, list[str]]:
    """Return determinant, orthogonality error, and warning messages."""
    det_r = float(np.linalg.det(rotation))
    orth_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro"))
    warnings = []

    if abs(det_r - 1.0) > 5e-3:
        warnings.append(
            "det(R) is not close to 1. Check frame direction, row/column convention, or matrix validity."
        )
    if orth_error > 5e-3:
        warnings.append(
            "R.T @ R deviates from I. Check numerical precision, matrix construction, or axis ordering."
        )

    return det_r, orth_error, warnings


def add_axis(
    fig: go.Figure,
    origin: np.ndarray,
    direction: np.ndarray,
    axis_length: float,
    color: str,
    name: str,
    label: str,
    showlegend: bool,
) -> None:
    """Add a single colored axis arrow with label to the figure."""
    origin = np.asarray(origin, dtype=float)
    unit_direction = np.asarray(direction, dtype=float)
    unit_direction = unit_direction / np.linalg.norm(unit_direction)

    end = origin + unit_direction * axis_length
    label_pos = end + unit_direction * axis_length * 0.12
    cone_size = max(axis_length * 0.16, 0.08)

    fig.add_trace(
        go.Scatter3d(
            x=[origin[0], end[0]],
            y=[origin[1], end[1]],
            z=[origin[2], end[2]],
            mode="lines",
            line=dict(color=color, width=8),
            name=name,
            legendgroup=name,
            showlegend=showlegend,
            hovertemplate=f"{name}<extra></extra>",
        )
    )

    fig.add_trace(
        go.Cone(
            x=[end[0]],
            y=[end[1]],
            z=[end[2]],
            u=[unit_direction[0]],
            v=[unit_direction[1]],
            w=[unit_direction[2]],
            sizemode="absolute",
            sizeref=cone_size,
            anchor="tip",
            colorscale=[[0.0, color], [1.0, color]],
            showscale=False,
            hoverinfo="skip",
            showlegend=False,
            legendgroup=name,
        )
    )

    fig.add_trace(
        go.Scatter3d(
            x=[label_pos[0]],
            y=[label_pos[1]],
            z=[label_pos[2]],
            mode="text",
            text=[label],
            textfont=dict(color=color, size=12),
            hoverinfo="skip",
            showlegend=False,
            legendgroup=name,
        )
    )


def add_frame(
    fig: go.Figure,
    rotation: np.ndarray,
    translation: np.ndarray,
    frame_name: str,
    axis_length: float,
    show_origin_legend: bool = True,
) -> None:
    """Add a coordinate frame origin, labels, and XYZ axes."""
    origin = np.asarray(translation, dtype=float)

    fig.add_trace(
        go.Scatter3d(
            x=[origin[0]],
            y=[origin[1]],
            z=[origin[2]],
            mode="markers+text",
            marker=dict(size=6, color="black"),
            text=[frame_name],
            textposition="top center",
            name=f"{frame_name} origin",
            legendgroup=frame_name,
            showlegend=show_origin_legend,
            hovertemplate=f"{frame_name} origin<extra></extra>",
        )
    )

    for axis_name, axis_basis in AXIS_VECTORS.items():
        direction = rotation @ axis_basis
        add_axis(
            fig=fig,
            origin=origin,
            direction=direction,
            axis_length=axis_length,
            color=AXIS_COLORS[axis_name],
            name=f"{frame_name} +{axis_name}",
            label=f"{frame_name} +{axis_name}",
            showlegend=True,
        )


def camera_from_direction(
    direction: np.ndarray,
    distance: float = 2.6,
    up_hint: np.ndarray | None = None,
) -> dict:
    """Create a Plotly camera aligned with the given viewing direction."""
    view_dir = np.asarray(direction, dtype=float)
    if np.linalg.norm(view_dir) < 1e-12:
        raise ValueError("Camera direction must be non-zero.")

    view_dir = view_dir / np.linalg.norm(view_dir)
    up = np.array([0.0, 0.0, 1.0]) if up_hint is None else np.asarray(up_hint, dtype=float)
    if np.linalg.norm(np.cross(view_dir, up)) < 1e-6:
        up = np.array([0.0, 1.0, 0.0])
    if np.linalg.norm(np.cross(view_dir, up)) < 1e-6:
        up = np.array([1.0, 0.0, 0.0])

    up = up - np.dot(up, view_dir) * view_dir
    up = up / np.linalg.norm(up)
    eye = view_dir * distance

    return {
        "eye": {"x": float(eye[0]), "y": float(eye[1]), "z": float(eye[2])},
        "up": {"x": float(up[0]), "y": float(up[1]), "z": float(up[2])},
        "center": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


def make_view_buttons(source_rotation: np.ndarray) -> list[dict]:
    """Build Plotly dropdown buttons for preset target/source-aligned views."""
    buttons = [
        dict(
            label="Default Oblique",
            method="relayout",
            args=[{"scene.camera": camera_from_direction(np.array([1.5, 1.2, 0.9]), distance=2.8)}],
        )
    ]

    for prefix, rotation in (("Target", np.eye(3)), ("Source", source_rotation)):
        for axis_name, sign in (
            ("+X", 1.0),
            ("-X", -1.0),
            ("+Y", 1.0),
            ("-Y", -1.0),
            ("+Z", 1.0),
            ("-Z", -1.0),
        ):
            basis = AXIS_VECTORS[axis_name[-1]]
            direction = rotation @ (sign * basis)
            buttons.append(
                dict(
                    label=f"{prefix} {axis_name}",
                    method="relayout",
                    args=[{"scene.camera": camera_from_direction(direction)}],
                )
            )

    return buttons


def build_figure(transform: np.ndarray, source_name: str, target_name: str) -> go.Figure:
    """Create the interactive Plotly figure for the two frames."""
    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    translation_norm = float(np.linalg.norm(translation))
    axis_length = max(0.5, 0.35 * translation_norm + 0.6)

    fig = go.Figure()
    add_frame(fig, np.eye(3), np.zeros(3), target_name, axis_length)
    add_frame(fig, rotation, translation, source_name, axis_length)

    points = [np.zeros(3), translation]
    for frame_rotation, frame_origin in ((np.eye(3), np.zeros(3)), (rotation, translation)):
        for axis_basis in AXIS_VECTORS.values():
            points.append(frame_origin + frame_rotation @ axis_basis * axis_length * 1.2)

    point_array = np.vstack(points)
    minima = point_array.min(axis=0)
    maxima = point_array.max(axis=0)
    center = (minima + maxima) / 2.0
    radius = max(1.0, float(np.max(maxima - minima)) * 0.65)

    fig.update_layout(
        title=(
            f"Extrinsic Visualizer: {source_name} in {target_name} "
            f"(p_{target_name} = T_{target_name}_{source_name} @ p_{source_name})"
        ),
        template="plotly_white",
        width=1200,
        height=820,
        legend=dict(
            x=1.02,
            y=1.0,
            xanchor="left",
            yanchor="top",
            bgcolor="rgba(255,255,255,0.8)",
            bordercolor="rgba(0,0,0,0.2)",
            borderwidth=1,
        ),
        margin=dict(l=0, r=220, b=0, t=90),
        scene=dict(
            aspectmode="cube",
            xaxis=dict(
                title="X",
                range=[center[0] - radius, center[0] + radius],
                backgroundcolor="rgba(245,245,245,1.0)",
                gridcolor="rgba(180,180,180,0.35)",
                zerolinecolor="rgba(120,120,120,0.55)",
            ),
            yaxis=dict(
                title="Y",
                range=[center[1] - radius, center[1] + radius],
                backgroundcolor="rgba(245,245,245,1.0)",
                gridcolor="rgba(180,180,180,0.35)",
                zerolinecolor="rgba(120,120,120,0.55)",
            ),
            zaxis=dict(
                title="Z",
                range=[center[2] - radius, center[2] + radius],
                backgroundcolor="rgba(245,245,245,1.0)",
                gridcolor="rgba(180,180,180,0.35)",
                zerolinecolor="rgba(120,120,120,0.55)",
            ),
            camera=camera_from_direction(np.array([1.5, 1.2, 0.9]), distance=2.8),
        ),
        updatemenus=[
            dict(
                type="dropdown",
                direction="down",
                x=0.02,
                y=1.05,
                xanchor="left",
                yanchor="top",
                showactive=True,
                buttons=make_view_buttons(rotation),
            )
        ],
        annotations=[
            dict(
                x=0.02,
                y=1.105,
                xref="paper",
                yref="paper",
                text="View Presets",
                showarrow=False,
                font=dict(size=13),
                align="left",
            )
        ],
    )

    return fig


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize and validate a 4x4 extrinsic matrix between two sensor frames.",
        epilog=USAGE_EXAMPLES,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument(
        "--matrix",
        type=str,
        help="4x4 matrix literal. Supports JSON lists or plain text such as '1 0 0 0; 0 1 0 0; ...'.",
    )
    input_group.add_argument(
        "--matrix-file",
        type=str,
        help="Path to a .txt or .json file containing the matrix.",
    )
    parser.add_argument("--inverse", action="store_true", help="Invert the input matrix before visualization.")
    parser.add_argument("--source", default="source", help="Source frame name, e.g. LiDAR.")
    parser.add_argument("--target", default="target", help="Target frame name, e.g. IMU.")
    parser.add_argument(
        "--output",
        default="extrinsic_visualizer.html",
        help="Output HTML path. The generated file can be opened directly in a browser.",
    )

    args = parser.parse_args()

    np.set_printoptions(precision=6, suppress=True)

    try:
        input_matrix = load_matrix(args.matrix, args.matrix_file)
        if input_matrix.shape != (4, 4):
            raise ValueError(f"Expected a 4x4 matrix, got shape {input_matrix.shape}.")
        transform = np.linalg.inv(input_matrix) if args.inverse else input_matrix.copy()
    except (FileNotFoundError, json.JSONDecodeError, np.linalg.LinAlgError, ValueError) as exc:
        parser.error(str(exc))

    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    det_r, orth_error, warnings = rotation_diagnostics(rotation)

    print("Input matrix:")
    print(input_matrix)
    print()

    if args.inverse:
        print("Display matrix after --inverse:")
        print(transform)
        print()
    else:
        print("Display matrix:")
        print(transform)
        print()

    print(f"source: {args.source}")
    print(f"target: {args.target}")
    print(f"translation t: {translation}")
    print(f"det(R): {det_r:.8f}")
    print(f"||R.T @ R - I||_F: {orth_error:.8e}")
    for warning in warnings:
        print(f"WARNING: {warning}")

    figure = build_figure(transform, args.source, args.target)
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.write_html(str(output_path), include_plotlyjs=True, full_html=True)
    print(f"HTML written to: {output_path}")


if __name__ == "__main__":
    main()
