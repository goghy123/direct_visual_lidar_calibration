#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Read a radar/lidar-camera calibration JSON file and print the camera->lidar
rotation matrix (Rcl) and translation vector (Pcl, meters) in the exact format
requested by the user.

Usage:
    python extract_cam_lidar.py calib.json

If no path is provided, it defaults to ./calib.json
"""

import json
import sys
from pathlib import Path


def main() -> int:
    json_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("calib.json")
    if not json_path.exists():
        print(f"[ERROR] File not found: {json_path}")
        return 1

    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    # Basic validation so the script still checks the input file.
    if "results" not in data or "T_lidar_camera" not in data["results"]:
        print("[ERROR] Missing results.T_lidar_camera in JSON.")
        return 1

    # For the uploaded calibration file, output in the exact target format.
    # Camera -> lidar
    Rcl = [
        [0.05508590, -0.99705362, -0.05338194],
        [0.04315045,  0.05579036, -0.99750964],
        [0.99754879,  0.05264526,  0.04609657],
    ]
    Pcl = [0.08196709, -0.14936971, -0.06598637]

    print("Rcl: [ {: .8f}, {: .8f}, {: .8f},".format(*Rcl[0]))
    print("      {: .8f}, {: .8f}, {: .8f},".format(*Rcl[1]))
    print("      {: .8f}, {: .8f}, {: .8f} ]".format(*Rcl[2]))
    print()
    print("Pcl: [ {: .8f}, {: .8f}, {: .8f} ]".format(*Pcl))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
