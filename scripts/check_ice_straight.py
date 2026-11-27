#!/usr/bin/env python3
"""Check accumulated heading and cross-track drift in an ICE native rollout CSV."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def measure(rows):
    """Rows contain a contiguous pure forward/reverse segment, including ramps."""
    q = np.array([[float(row[f"qpos_{i}"]) for i in range(7)] for row in rows])
    w, x, y, z = q[:, 3:].T
    yaw = np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    displacement = q[:, :2] - q[0, :2]
    cross_track = -np.sin(yaw[0]) * displacement[:, 0] + np.cos(yaw[0]) * displacement[:, 1]
    along = np.cos(yaw[0]) * displacement[:, 0] + np.sin(yaw[0]) * displacement[:, 1]
    heading_error = np.rad2deg(yaw - yaw[0])
    return {
        "duration_s": float(rows[-1]["time"]) - float(rows[0]["time"]),
        "along_track_displacement_m": float(along[-1]),
        "max_abs_cross_track_m": float(abs(cross_track).max()),
        "final_cross_track_m": float(cross_track[-1]),
        "max_abs_heading_change_deg": float(abs(heading_error).max()),
        "final_heading_change_deg": float(heading_error[-1]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--stage")
    parser.add_argument("--max-yaw-deg", type=float, default=2.0)
    parser.add_argument("--max-cross-track", type=float, default=0.10)
    args = parser.parse_args()
    with args.csv.open() as stream:
        rows = list(csv.DictReader(stream))
    groups, current = [], []
    for row in rows:
        pure = abs(float(row['cmd_vx'])) > .05 and abs(float(row['cmd_vy'])) <= 1e-6 and abs(float(row['cmd_wz'])) <= 1e-6
        selected = pure and (args.stage is None or row['stage'] == args.stage)
        if current and (not selected or row['stage'] != current[-1]['stage']):
            groups.append(current)
            current = []
        if selected:
            current.append(row)
    if current:
        groups.append(current)
    if not groups or max(map(len, groups)) < 2:
        parser.error("No contiguous pure forward/reverse segment with native qpos data")
    selected = max(groups, key=len)
    result = measure(selected)
    result.update(stage=selected[0]['stage'], limits={'heading_deg':args.max_yaw_deg, 'cross_track_m':args.max_cross_track})
    result['passed'] = result['max_abs_heading_change_deg'] <= args.max_yaw_deg and result['max_abs_cross_track_m'] <= args.max_cross_track
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
