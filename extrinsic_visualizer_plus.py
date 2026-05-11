
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Extrinsic Visualizer + Report.

Convention:
    p_target = T_target_source @ p_source

Default Euler convention:
    ZYX yaw-pitch-roll, column vector convention:
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

Supported input formats:
    1. 4x4 matrix, nested or flat 16 numbers
    2. ROS tf style: tx ty tz qx qy qz qw
    3. tx ty tz qw qx qy qz
    4. tx ty tz roll pitch yaw, degrees or radians
"""
from __future__ import annotations
import argparse
import html
import json
import math
import re
import webbrowser
from pathlib import Path

import numpy as np
import plotly.graph_objects as go


AXES = ["X", "Y", "Z"]

COLORS = {
    "X": "#e74c3c",
    "Y": "#2ecc71",
    "Z": "#3498db",
}


UNIT_SCALE_TO_M = {
    "m": 1.0,
    "cm": 0.01,
    "mm": 0.001,
}


def extract_numbers(text: str) -> np.ndarray:
    nums = re.findall(
        r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?",
        text,
    )
    return np.array([float(x) for x in nums], dtype=float)


def parse_json_like_matrix(text: str) -> np.ndarray | None:
    """Try parsing normal JSON matrix first."""
    try:
        obj = json.loads(text.strip())
        arr = np.array(obj, dtype=float)
        if arr.shape == (4, 4):
            return arr
        if arr.size == 16:
            return arr.reshape(4, 4)
    except Exception:
        return None
    return None


def quat_xyzw_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Quaternion norm is zero.")
    qx, qy, qz, qw = q / n

    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=float,
    )


def euler_zyx_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Angles are radians."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)

    return Rz @ Ry @ Rx


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def parse_transform_text(text: str, input_format: str) -> np.ndarray:
    text = text.strip()

    if input_format in ("auto", "matrix4x4", "flat16"):
        m = parse_json_like_matrix(text)
        if m is not None:
            return m

    values = extract_numbers(text)

    if input_format == "auto":
        if values.size == 16:
            return values.reshape(4, 4)
        if values.size == 7:
            # ROS commonly prints translation x y z and rotation x y z w.
            tx, ty, tz, qx, qy, qz, qw = values
            return make_T(quat_xyzw_to_rot(qx, qy, qz, qw), np.array([tx, ty, tz]))
        if values.size == 6:
            # Convenient default: tx ty tz roll pitch yaw in degrees.
            tx, ty, tz, roll, pitch, yaw = values
            R = euler_zyx_to_rot(
                math.radians(roll),
                math.radians(pitch),
                math.radians(yaw),
            )
            return make_T(R, np.array([tx, ty, tz]))
        raise ValueError(
            "Auto format supports 16 numbers as matrix, 7 numbers as "
            "tx ty tz qx qy qz qw, or 6 numbers as tx ty tz roll pitch yaw in degrees."
        )

    if input_format in ("matrix4x4", "flat16"):
        if values.size != 16:
            raise ValueError(f"{input_format} requires 16 numbers, got {values.size}.")
        return values.reshape(4, 4)

    if input_format == "tq_xyzw":
        if values.size != 7:
            raise ValueError("tq_xyzw requires: tx ty tz qx qy qz qw")
        tx, ty, tz, qx, qy, qz, qw = values
        return make_T(quat_xyzw_to_rot(qx, qy, qz, qw), np.array([tx, ty, tz]))

    if input_format == "tq_wxyz":
        if values.size != 7:
            raise ValueError("tq_wxyz requires: tx ty tz qw qx qy qz")
        tx, ty, tz, qw, qx, qy, qz = values
        return make_T(quat_xyzw_to_rot(qx, qy, qz, qw), np.array([tx, ty, tz]))

    if input_format == "xyzrpy_deg":
        if values.size != 6:
            raise ValueError("xyzrpy_deg requires: tx ty tz roll pitch yaw, angle unit degree")
        tx, ty, tz, roll, pitch, yaw = values
        R = euler_zyx_to_rot(
            math.radians(roll),
            math.radians(pitch),
            math.radians(yaw),
        )
        return make_T(R, np.array([tx, ty, tz]))

    if input_format == "xyzrpy_rad":
        if values.size != 6:
            raise ValueError("xyzrpy_rad requires: tx ty tz roll pitch yaw, angle unit radian")
        tx, ty, tz, roll, pitch, yaw = values
        R = euler_zyx_to_rot(roll, pitch, yaw)
        return make_T(R, np.array([tx, ty, tz]))

    raise ValueError(f"Unsupported input format: {input_format}")


def load_transform(input_arg: str | None, file_arg: str | None, input_format: str) -> np.ndarray:
    if input_arg:
        return parse_transform_text(input_arg, input_format)
    if file_arg:
        return parse_transform_text(Path(file_arg).read_text(encoding="utf-8"), input_format)
    raise ValueError("Please provide --input or --file.")


def normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < eps:
        raise ValueError(f"Zero-length vector cannot be normalized: {v}")
    return v / n


def rotation_diagnostics(R: np.ndarray) -> dict:
    ortho_err = np.linalg.norm(R.T @ R - np.eye(3), ord="fro")
    det = np.linalg.det(R)
    return {
        "orthogonality_error": float(ortho_err),
        "determinant": float(det),
    }


def rotmat_to_euler_zyx(R: np.ndarray) -> tuple[float, float, float]:
    """
    Return roll, pitch, yaw in radians.

    Convention:
        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-9

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


def rotmat_to_quat_xyzw(R: np.ndarray) -> tuple[float, float, float, float]:
    """Convert rotation matrix to quaternion in qx qy qz qw order."""
    tr = float(np.trace(R))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=float)
    q = q / np.linalg.norm(q)
    return tuple(float(x) for x in q)


def rotmat_to_axis_angle(R: np.ndarray) -> tuple[np.ndarray, float]:
    cos_angle = (np.trace(R) - 1.0) / 2.0
    cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
    angle = math.acos(cos_angle)

    if abs(angle) < 1e-12:
        return np.array([1.0, 0.0, 0.0]), 0.0

    axis = np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ],
        dtype=float,
    ) / (2.0 * math.sin(angle))

    axis = normalize(axis)
    return axis, angle


def fmt_float(x: float, ndigits: int = 6) -> str:
    if abs(x) < 5 * 10 ** (-(ndigits + 1)):
        x = 0.0
    return f"{x:.{ndigits}f}"


def matrix_to_html_table(M: np.ndarray, ndigits: int = 6) -> str:
    rows = []
    for row in M:
        tds = "".join(f"<td>{fmt_float(float(v), ndigits)}</td>" for v in row)
        rows.append(f"<tr>{tds}</tr>")
    return "<table class='matrix'>" + "".join(rows) + "</table>"


def make_report_html(T: np.ndarray, target_name: str, source_name: str, unit: str) -> str:
    R = T[:3, :3]
    t = T[:3, 3]

    diag = rotation_diagnostics(R)
    roll, pitch, yaw = rotmat_to_euler_zyx(R)
    qx, qy, qz, qw = rotmat_to_quat_xyzw(R)
    axis, angle = rotmat_to_axis_angle(R)

    warning = ""
    if abs(diag["determinant"] - 1.0) > 1e-3 or diag["orthogonality_error"] > 1e-3:
        warning = (
            "<div class='warning'>WARNING: R 可能不是合法旋转矩阵。"
            "请检查矩阵方向、行列约定、单位或数值精度。</div>"
        )

    t_norm = float(np.linalg.norm(t))

    return f"""
    <div class="report">
      <h1>Extrinsic Visualization Report</h1>
      <div class="subtitle">
        Convention:
        <code>p_{html.escape(target_name)} = T_{html.escape(target_name)}_{html.escape(source_name)} · p_{html.escape(source_name)}</code>
      </div>

      {warning}

      <div class="grid">
        <div class="card">
          <h2>4×4 外参矩阵</h2>
          {matrix_to_html_table(T)}
        </div>

        <div class="card">
          <h2>平移向量，单位：m</h2>
          <p><code>t = [{fmt_float(t[0])}, {fmt_float(t[1])}, {fmt_float(t[2])}] m</code></p>
          <p>含义：<b>{html.escape(source_name)}</b> 坐标系原点在 <b>{html.escape(target_name)}</b> 坐标系下的位置。</p>
          <p>平移模长：<code>{fmt_float(t_norm)} m</code></p>
          <p class="note">输入平移单位参数：<code>{html.escape(unit)}</code>，已统一换算到米。</p>
        </div>

        <div class="card">
          <h2>旋转角度表达，单位：degree</h2>
          <p>Euler ZYX / RPY 约定：</p>
          <p><code>R = Rz(yaw) · Ry(pitch) · Rx(roll)</code></p>
          <table>
            <tr><th>roll around X</th><td>{fmt_float(math.degrees(roll))}°</td></tr>
            <tr><th>pitch around Y</th><td>{fmt_float(math.degrees(pitch))}°</td></tr>
            <tr><th>yaw around Z</th><td>{fmt_float(math.degrees(yaw))}°</td></tr>
          </table>
        </div>

        <div class="card">
          <h2>其他旋转表达</h2>
          <p>Quaternion, ROS xyzw:</p>
          <p><code>[{fmt_float(qx)}, {fmt_float(qy)}, {fmt_float(qz)}, {fmt_float(qw)}]</code></p>
          <p>Axis-angle:</p>
          <p><code>axis=[{fmt_float(axis[0])}, {fmt_float(axis[1])}, {fmt_float(axis[2])}], angle={fmt_float(math.degrees(angle))}°</code></p>
        </div>

        <div class="card">
          <h2>旋转矩阵检查</h2>
          <table>
            <tr><th>det(R)</th><td>{fmt_float(diag["determinant"], 9)}</td><td>理想值：+1</td></tr>
            <tr><th>||RᵀR-I||F</th><td>{fmt_float(diag["orthogonality_error"], 9)}</td><td>理想值：0</td></tr>
          </table>
        </div>
      </div>
    </div>
    """


def add_axis(
    fig: go.Figure,
    origin: np.ndarray,
    direction: np.ndarray,
    length: float,
    axis_name: str,
    frame_name: str,
    legend_group: str,
    line_width: int,
    opacity: float,
):
    direction = normalize(direction)
    end = origin + direction * length
    color = COLORS[axis_name]

    fig.add_trace(
        go.Scatter3d(
            x=[origin[0], end[0]],
            y=[origin[1], end[1]],
            z=[origin[2], end[2]],
            mode="lines",
            name=f"{frame_name} +{axis_name}",
            legendgroup=legend_group,
            line=dict(color=color, width=line_width),
            opacity=opacity,
            showlegend=True,
        )
    )

    fig.add_trace(
        go.Cone(
            x=[end[0]],
            y=[end[1]],
            z=[end[2]],
            u=[direction[0]],
            v=[direction[1]],
            w=[direction[2]],
            anchor="tip",
            sizemode="absolute",
            sizeref=length * 0.12,
            colorscale=[[0, color], [1, color]],
            showscale=False,
            opacity=opacity,
            name=f"{frame_name} +{axis_name} arrow",
            legendgroup=legend_group,
            showlegend=False,
            hoverinfo="skip",
        )
    )

    label_pos = origin + direction * length * 1.13

    fig.add_trace(
        go.Scatter3d(
            x=[label_pos[0]],
            y=[label_pos[1]],
            z=[label_pos[2]],
            mode="text",
            text=[f"{frame_name}<br>+{axis_name}"],
            textfont=dict(color=color, size=13),
            name=f"{frame_name} +{axis_name} label",
            legendgroup=legend_group,
            showlegend=False,
            hoverinfo="skip",
        )
    )


def add_frame(
    fig: go.Figure,
    frame_name: str,
    origin: np.ndarray,
    R: np.ndarray,
    length: float,
    legend_group: str,
    line_width: int = 8,
    opacity: float = 1.0,
):
    fig.add_trace(
        go.Scatter3d(
            x=[origin[0]],
            y=[origin[1]],
            z=[origin[2]],
            mode="markers+text",
            marker=dict(size=5),
            text=[f"{frame_name}<br>origin"],
            textposition="top center",
            name=f"{frame_name} origin",
            legendgroup=legend_group,
            showlegend=True,
        )
    )

    for i, axis_name in enumerate(AXES):
        add_axis(
            fig=fig,
            origin=origin,
            direction=R[:, i],
            length=length,
            axis_name=axis_name,
            frame_name=frame_name,
            legend_group=legend_group,
            line_width=line_width,
            opacity=opacity,
        )


def camera_from_direction(direction: np.ndarray, up_hint: np.ndarray, distance: float) -> dict:
    d = normalize(direction)
    up = np.array(up_hint, dtype=float)
    up = up - np.dot(up, d) * d

    if np.linalg.norm(up) < 1e-8:
        for candidate in (
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 1.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
        ):
            up = candidate - np.dot(candidate, d) * d
            if np.linalg.norm(up) >= 1e-8:
                break

    up = normalize(up)
    eye = d * distance

    return {
        "eye": {"x": float(eye[0]), "y": float(eye[1]), "z": float(eye[2])},
        "up": {"x": float(up[0]), "y": float(up[1]), "z": float(up[2])},
    }


def make_view_buttons(target_name: str, source_name: str, R_source_in_target: np.ndarray, camera_distance: float):
    frames = [(target_name, np.eye(3)), (source_name, R_source_in_target)]

    buttons = [
        dict(
            label="默认斜视",
            method="relayout",
            args=[{"scene.camera": {"eye": {"x": 1.7, "y": 1.7, "z": 1.3}, "up": {"x": 0, "y": 0, "z": 1}}}],
        )
    ]

    for frame_name, R in frames:
        for i, axis_name in enumerate(AXES):
            for sign, sign_name in [(1.0, "+"), (-1.0, "-")]:
                direction = sign * R[:, i]
                up_hint = R[:, 2] if axis_name in ("X", "Y") else R[:, 1]
                cam = camera_from_direction(direction=direction, up_hint=up_hint, distance=camera_distance)
                buttons.append(
                    dict(
                        label=f"沿 {frame_name} {sign_name}{axis_name} 看",
                        method="relayout",
                        args=[{"scene.camera": cam}],
                    )
                )
    return buttons


def compute_scene_ranges(points: np.ndarray, pad_ratio: float = 0.25):
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) / 2.0
    span = float(np.max(maxs - mins))
    if span < 1e-6:
        span = 1.0
    half = span * (0.5 + pad_ratio)
    return [[float(center[i] - half), float(center[i] + half)] for i in range(3)]


def build_figure(T: np.ndarray, target_name: str, source_name: str, axis_length: float) -> go.Figure:
    R = T[:3, :3]
    t = T[:3, 3]

    fig = go.Figure()

    target_origin = np.zeros(3)
    target_R = np.eye(3)

    add_frame(
        fig=fig,
        frame_name=target_name,
        origin=target_origin,
        R=target_R,
        length=axis_length,
        legend_group=target_name,
        line_width=8,
        opacity=1.0,
    )

    add_frame(
        fig=fig,
        frame_name=source_name,
        origin=t,
        R=R,
        length=axis_length,
        legend_group=source_name,
        line_width=6,
        opacity=0.88,
    )

    pts = [target_origin, t]
    for i in range(3):
        pts.append(target_origin + target_R[:, i] * axis_length)
        pts.append(t + R[:, i] * axis_length)
    pts = np.vstack(pts)

    x_range, y_range, z_range = compute_scene_ranges(pts)
    buttons = make_view_buttons(target_name, source_name, R, camera_distance=2.2)

    title = f"3D Coordinate Frames: p_{target_name} = T_{target_name}_{source_name} · p_{source_name}"

    fig.update_layout(
        title=title,
        width=1150,
        height=780,
        scene=dict(
            xaxis=dict(title=f"{target_name} X / m", range=x_range, showbackground=True),
            yaxis=dict(title=f"{target_name} Y / m", range=y_range, showbackground=True),
            zaxis=dict(title=f"{target_name} Z / m", range=z_range, showbackground=True),
            aspectmode="cube",
            camera=dict(eye=dict(x=1.7, y=1.7, z=1.3), up=dict(x=0, y=0, z=1)),
        ),
        legend=dict(title="Frame / Axis", x=0.01, y=0.98, bgcolor="rgba(255,255,255,0.75)"),
        margin=dict(l=0, r=0, b=0, t=70),
        updatemenus=[
            dict(
                type="dropdown",
                direction="down",
                x=0.76,
                y=1.06,
                xanchor="left",
                yanchor="top",
                buttons=buttons,
                showactive=True,
            )
        ],
    )
    return fig


def write_html_page(fig: go.Figure, report_html: str, output_path: Path):
    fig_html = fig.to_html(full_html=False, include_plotlyjs="cdn")

    css = """
    <style>
      body {
        margin: 0;
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Arial, sans-serif;
        background: #f6f7f9;
        color: #1f2937;
      }
      .report {
        padding: 24px 32px 10px 32px;
      }
      h1 {
        margin: 0 0 8px 0;
        font-size: 26px;
      }
      h2 {
        margin-top: 0;
        font-size: 17px;
      }
      .subtitle {
        color: #4b5563;
        margin-bottom: 16px;
      }
      .grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
        gap: 14px;
      }
      .card {
        background: white;
        border-radius: 14px;
        padding: 16px;
        box-shadow: 0 4px 18px rgba(15, 23, 42, 0.08);
        overflow-x: auto;
      }
      table {
        border-collapse: collapse;
        width: 100%;
      }
      th, td {
        padding: 7px 9px;
        border-bottom: 1px solid #e5e7eb;
        text-align: left;
        white-space: nowrap;
      }
      .matrix td {
        font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
        text-align: right;
      }
      code {
        background: #f3f4f6;
        padding: 2px 5px;
        border-radius: 6px;
        font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      }
      .note {
        color: #6b7280;
        font-size: 13px;
      }
      .warning {
        background: #fff7ed;
        color: #9a3412;
        border: 1px solid #fed7aa;
        border-radius: 12px;
        padding: 12px 14px;
        margin: 14px 0;
      }
      .plot {
        margin: 8px 24px 28px 24px;
        background: white;
        border-radius: 16px;
        box-shadow: 0 4px 18px rgba(15, 23, 42, 0.08);
      }
    </style>
    """

    full_html = f"""
    <!doctype html>
    <html>
    <head>
      <meta charset="utf-8"/>
      <title>Extrinsic Visualizer</title>
      {css}
    </head>
    <body>
      {report_html}
      <div class="plot">
        {fig_html}
      </div>
    </body>
    </html>
    """

    output_path.write_text(full_html, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Visualize and inspect a 4x4 extrinsic matrix.")

    parser.add_argument("--input", type=str, default=None, help="Input transform text.")
    parser.add_argument("--file", type=str, default=None, help="Input txt/json file.")

    parser.add_argument(
        "--format",
        type=str,
        default="auto",
        choices=["auto", "matrix4x4", "flat16", "tq_xyzw", "tq_wxyz", "xyzrpy_deg", "xyzrpy_rad"],
        help=(
            "Input format. auto: 16 numbers=matrix, "
            "7 numbers=tx ty tz qx qy qz qw, "
            "6 numbers=tx ty tz roll pitch yaw in degrees."
        ),
    )

    parser.add_argument(
        "--translation-unit",
        type=str,
        default="m",
        choices=["m", "cm", "mm"],
        help="Unit of input translation. Output and plot are always in meters.",
    )

    parser.add_argument("--source", type=str, default="LiDAR", help="Source frame name.")
    parser.add_argument("--target", type=str, default="IMU", help="Target/reference frame name.")
    parser.add_argument("--axis-length", type=float, default=1.0, help="Axis length in meters.")
    parser.add_argument("--inverse", action="store_true", help="Invert the input matrix before visualization.")
    parser.add_argument("--output", type=str, default="extrinsic_visualizer.html", help="Output HTML file.")
    parser.add_argument("--open", action="store_true", help="Open output HTML in browser.")

    args = parser.parse_args()

    T = load_transform(args.input, args.file, args.format)

    # Convert translation part to meters before inverse, because the matrix should be physically consistent.
    T[:3, 3] *= UNIT_SCALE_TO_M[args.translation_unit]

    if args.inverse:
        T = np.linalg.inv(T)

    R = T[:3, :3]
    t = T[:3, 3]
    diag = rotation_diagnostics(R)
    roll, pitch, yaw = rotmat_to_euler_zyx(R)

    print("========== Extrinsic ==========")
    print(f"Convention: p_{args.target} = T_{args.target}_{args.source} @ p_{args.source}")
    print(T)

    print("\n========== Translation, meter ==========")
    print(f"t = [{t[0]:.9f}, {t[1]:.9f}, {t[2]:.9f}] m")
    print(f"|t| = {np.linalg.norm(t):.9f} m")

    print("\n========== Euler ZYX, degree ==========")
    print("R = Rz(yaw) @ Ry(pitch) @ Rx(roll)")
    print(f"roll  around X = {math.degrees(roll):.9f} deg")
    print(f"pitch around Y = {math.degrees(pitch):.9f} deg")
    print(f"yaw   around Z = {math.degrees(yaw):.9f} deg")

    print("\n========== Diagnostics ==========")
    print(f"det(R) = {diag['determinant']:.9f}")
    print(f"||R.T @ R - I||_F = {diag['orthogonality_error']:.9e}")

    if abs(diag["determinant"] - 1.0) > 1e-3 or diag["orthogonality_error"] > 1e-3:
        print("WARNING: R may not be a valid rotation matrix.")

    fig = build_figure(
        T=T,
        target_name=args.target,
        source_name=args.source,
        axis_length=args.axis_length,
    )

    report_html = make_report_html(
        T=T,
        target_name=args.target,
        source_name=args.source,
        unit=args.translation_unit,
    )

    out = Path(args.output).resolve()
    write_html_page(fig, report_html, out)
    print(f"\nSaved: {out}")

    if args.open:
        webbrowser.open(out.as_uri())


if __name__ == "__main__":
    main()