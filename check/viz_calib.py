import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import plotly.graph_objects as go
from pathlib import Path

    ######################################################################
    # 用法：终端输入 python viz_calib.py calib.json --out calib_frames.html #
    ######################################################################

def quat_to_rot(qx, qy, qz, qw):
    # scipy uses (x,y,z,w)
    return R.from_quat([qx, qy, qz, qw]).as_matrix()

def make_T(t, q):
    tx, ty, tz = t
    qx, qy, qz, qw = q
    Rot = quat_to_rot(qx, qy, qz, qw)
    T = np.eye(4)
    T[:3, :3] = Rot
    T[:3, 3] = [tx, ty, tz]
    return T

def add_frame(fig, T, name, axis_len=0.2, width=6):
    # Draw xyz axes of frame located at T (w.r.t. world)
    origin = T[:3, 3]
    x_axis = origin + T[:3, 0] * axis_len
    y_axis = origin + T[:3, 1] * axis_len
    z_axis = origin + T[:3, 2] * axis_len

    def seg(a, b, color, label):
        fig.add_trace(go.Scatter3d(
            x=[a[0], b[0]], y=[a[1], b[1]], z=[a[2], b[2]],
            mode="lines",
            line=dict(width=width, color=color),
            name=f"{name}-{label}"
        ))

    # Plotly default colors: red/green/blue like convention
    seg(origin, x_axis, "red", "x")
    seg(origin, y_axis, "green", "y")
    seg(origin, z_axis, "blue", "z")

    # Label origin
    fig.add_trace(go.Scatter3d(
        x=[origin[0]], y=[origin[1]], z=[origin[2]],
        mode="markers+text",
        marker=dict(size=4),
        text=[name],
        textposition="top center",
        name=name
    ))

def main(calib_path: str, out_html: str = "calib_frames.html"):
    calib_path = Path(calib_path)
    data = json.loads(calib_path.read_text())

    t = data["results"]["T_lidar_camera"][:3]
    q = data["results"]["T_lidar_camera"][3:7]
    T_lidar_camera = make_T(t, q)

    # camera frame relative to lidar:
    T_camera_lidar = np.linalg.inv(T_lidar_camera)

    # Pretty prints
    rot = R.from_matrix(T_lidar_camera[:3, :3])
    euler = rot.as_euler("xyz", degrees=True)
    angle_axis = rot.as_rotvec()
    angle = np.linalg.norm(angle_axis)
    axis = angle_axis / angle if angle > 1e-12 else np.array([0, 0, 1])

    print("=== Parsed from calib.json ===")
    print("T_lidar_camera (4x4):\n", np.array_str(T_lidar_camera, precision=6, suppress_small=True))
    print("\nT_camera_lidar (inverse):\n", np.array_str(T_camera_lidar, precision=6, suppress_small=True))
    print("\nTranslation (m):", np.array(t))
    print("Quaternion (x,y,z,w):", np.array(q))
    print("Euler xyz (deg):", np.array_str(euler, precision=3))
    print(f"Axis-angle: angle={angle:.6f} rad, axis={np.array_str(axis, precision=6)}")

    # Visualization:
    # Choose world = LiDAR frame at origin.
    fig = go.Figure()
    add_frame(fig, np.eye(4), "LiDAR", axis_len=0.25, width=7)
    add_frame(fig, T_camera_lidar, "Camera", axis_len=0.25, width=7)

    # Draw a line between origins
    o_l = np.zeros(3)
    o_c = T_camera_lidar[:3, 3]
    fig.add_trace(go.Scatter3d(
        x=[o_l[0], o_c[0]], y=[o_l[1], o_c[1]], z=[o_l[2], o_c[2]],
        mode="lines",
        line=dict(width=2, color="gray"),
        name="origin link"
    ))

    fig.update_layout(
        title=f"Extrinsic frames (world=LiDAR). From {calib_path.name}",
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            aspectmode="data"
        ),
        legend=dict(itemsizing="constant")
    )

    fig.write_html(out_html, include_plotlyjs="cdn")
    print(f"\nSaved interactive visualization to: {out_html}")

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("calib_json", help="Path to calib.json")
    ap.add_argument("--out", default="calib_frames.html", help="Output html")
    args = ap.parse_args()
    main(args.calib_json, args.out)
