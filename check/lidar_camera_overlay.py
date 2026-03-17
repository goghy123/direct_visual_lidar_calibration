#!/usr/bin/env python3
import math
import os
import time
import argparse
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge

import message_filters
import tf2_ros
from tf2_ros import TransformException

import cv2


def load_yaml_config(path: str) -> dict:
    try:
        import yaml
    except ImportError:
        raise RuntimeError("缺少 PyYAML：请先执行 pip3 install pyyaml")

    if not os.path.exists(path):
        raise FileNotFoundError(f"找不到 YAML 配置文件：{path}")

    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    return cfg


def cfg_get(cfg: dict, keys, default=None):
    cur = cfg
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)


def tf_to_T(transform_stamped):
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def aruco_dict_from_id(dict_id: int):
    # 仅允许 4 种：4x4_1000 ~ 7x7_1000
    table = {
        0: "DICT_4X4_1000",
        1: "DICT_5X5_1000",
        2: "DICT_6X6_1000",
        3: "DICT_7X7_1000",
    }
    return table.get(int(dict_id), None)


class LidarCameraOverlayV2(Node):
    def __init__(self, yaml_cfg: dict, mode_override: str):
        super().__init__("lidar_camera_overlay_v2")
        self.bridge = CvBridge()
        self.yaml_cfg = yaml_cfg

        # ---- topics / frames ----
        self.declare_parameter("cloud_topic", cfg_get(yaml_cfg, ["topics", "cloud_topic"], "/velodyne_points"))
        self.declare_parameter("image_topic", cfg_get(yaml_cfg, ["topics", "image_topic"], "/camera/camera/color/image_raw"))
        self.declare_parameter("camera_info_topic", cfg_get(yaml_cfg, ["topics", "camera_info_topic"], "/camera/camera/color/camera_info"))
        self.declare_parameter("lidar_frame", cfg_get(yaml_cfg, ["frames", "lidar_frame"], "velodyne"))
        self.declare_parameter("camera_frame", cfg_get(yaml_cfg, ["frames", "camera_frame"], "camera_color_optical_frame"))

        # ---- sync/perf ----
        self.declare_parameter("slop_sec", float(cfg_get(yaml_cfg, ["sync_perf", "slop_sec"], 0.10)))
        self.declare_parameter("max_points", int(cfg_get(yaml_cfg, ["sync_perf", "max_points"], 80000)))
        self.declare_parameter("min_z_cam", float(cfg_get(yaml_cfg, ["sync_perf", "min_z_cam"], 0.1)))
        self.declare_parameter("max_z_cam", float(cfg_get(yaml_cfg, ["sync_perf", "max_z_cam"], 60.0)))
        self.declare_parameter("point_radius", int(cfg_get(yaml_cfg, ["sync_perf", "point_radius"], 2)))

        # ---- overlay coloring ----
        self.declare_parameter("color_mode", cfg_get(yaml_cfg, ["overlay", "color_mode"], "depth"))

        # ---- detector ----
        # "none" | "chessboard" | "aruco"
        self.declare_parameter("detector", cfg_get(yaml_cfg, ["detector", "type"], "none"))

        # chessboard params（YAML里用mm，内部仍用米，保持你的主流程不变）
        self.declare_parameter("chessboard_cols", int(cfg_get(yaml_cfg, ["chessboard", "cols"], 9)))
        self.declare_parameter("chessboard_rows", int(cfg_get(yaml_cfg, ["chessboard", "rows"], 6)))
        square_mm = float(cfg_get(yaml_cfg, ["chessboard", "square_mm"], 25.0))
        self.declare_parameter("chessboard_square", square_mm / 1000.0)  # meters

        # aruco params（YAML里用mm + dict_id）
        self.declare_parameter("aruco_dict_id", int(cfg_get(yaml_cfg, ["aruco", "dict_id"], 2)))
        marker_mm = float(cfg_get(yaml_cfg, ["aruco", "marker_length_mm"], 100.0))
        self.declare_parameter("aruco_marker_length", marker_mm / 1000.0)  # meters
        self.declare_parameter("aruco_prefer_id", int(cfg_get(yaml_cfg, ["aruco", "prefer_id"], -1)))

        # ---- saving ----
        self.declare_parameter("save_on_detect", bool(cfg_get(yaml_cfg, ["saving", "save_on_detect"], False)))
        self.declare_parameter("save_dir", cfg_get(yaml_cfg, ["saving", "save_dir"], "/tmp/lidar_overlay"))

        # 运行时模式覆盖（满足：终端只填 chess 或 aruco）
        self.mode_override = (mode_override or "").lower().strip()
        if self.mode_override == "chess":
            self.detector_override = "chessboard"
        elif self.mode_override == "aruco":
            self.detector_override = "aruco"
        else:
            self.detector_override = ""  # 不覆盖

        cloud_topic = self.get_parameter("cloud_topic").value
        image_topic = self.get_parameter("image_topic").value
        cam_info_topic = self.get_parameter("camera_info_topic").value

        self.pub = self.create_publisher(Image, "/lidar_overlay/image", 10)
        self.create_subscription(CameraInfo, cam_info_topic, self.on_camera_info, 10)

        self.fx = self.fy = self.cx = self.cy = None
        self.dist = None
        self.camera_ready = False

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # sync image + cloud
        self.sub_img = message_filters.Subscriber(self, Image, image_topic)
        self.sub_cloud = message_filters.Subscriber(self, PointCloud2, cloud_topic)
        slop = float(self.get_parameter("slop_sec").value)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_cloud], queue_size=10, slop=slop
        )
        self.sync.registerCallback(self.on_sync)

        self.get_logger().info("Started lidar_camera_overlay_v2.")
        self.get_logger().info(f"Sub: {image_topic}, {cloud_topic}, {cam_info_topic}")
        self.get_logger().info("Pub: /lidar_overlay/image")

        if self.detector_override:
            self.get_logger().info(f"[RUN MODE OVERRIDE] detector = {self.detector_override}")
        else:
            self.get_logger().info(f"[YAML MODE] detector = {self.get_parameter('detector').value}")

    def on_camera_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0]); self.fy = float(msg.k[4])
        self.cx = float(msg.k[2]); self.cy = float(msg.k[5])
        self.dist = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros((5,), dtype=np.float64)
        self.camera_ready = True

    def estimate_target_pose_and_polygon(self, img_bgr):
        """
        返回：
          pose: (R_cam_obj 3x3, t_cam_obj 3,)  —— object->camera
          poly_px: Nx2 float32 —— 靶标区域多边形（像素）
          label: str
        若没检测到则返回 (None, None, msg)
        """
        if self.detector_override:
            det = self.detector_override
        else:
            det = self.get_parameter("detector").value.lower().strip()

        if det == "none":
            return None, None, ""

        if det == "chessboard":
            cols = int(self.get_parameter("chessboard_cols").value)
            rows = int(self.get_parameter("chessboard_rows").value)
            square = float(self.get_parameter("chessboard_square").value)
            pattern = (cols, rows)

            gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            ok, corners = cv2.findChessboardCorners(gray, pattern)
            if not ok:
                return None, None, "Chessboard: NOT FOUND"

            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            cv2.drawChessboardCorners(img_bgr, pattern, corners, True)

            # object points on z=0 plane
            objp = np.zeros((rows*cols, 3), np.float64)
            objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square

            K = np.array([[self.fx, 0, self.cx],
                          [0, self.fy, self.cy],
                          [0, 0, 1]], dtype=np.float64)

            ok2, rvec, tvec = cv2.solvePnP(objp, corners, K, self.dist, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok2:
                return None, None, "Chessboard: PnP FAILED"

            R_cam_obj, _ = cv2.Rodrigues(rvec)
            t_cam_obj = tvec.reshape(3)

            # polygon = convex hull of all corners
            pts = corners.reshape(-1, 2).astype(np.float32)
            hull = cv2.convexHull(pts).reshape(-1, 2)

            return (R_cam_obj, t_cam_obj), hull, "Chessboard: FOUND"

        if det == "aruco":
            if not hasattr(cv2, "aruco"):
                return None, None, "Aruco: cv2.aruco NOT AVAILABLE"

            dict_id = int(self.get_parameter("aruco_dict_id").value)
            dict_name = aruco_dict_from_id(dict_id)
            if dict_name is None:
                return None, None, f"Aruco: dict_id out of range: {dict_id} (only 0..3)"

            dict_enum = getattr(cv2.aruco, dict_name, None)
            if dict_enum is None:
                return None, None, f"Aruco: OpenCV has no {dict_name}"

            dictionary = cv2.aruco.getPredefinedDictionary(dict_enum)
            params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, params)

            corners_list, ids, _ = detector.detectMarkers(img_bgr)
            if ids is None or len(ids) == 0:
                return None, None, f"Aruco: NOT FOUND ({dict_name})"

            prefer = int(self.get_parameter("aruco_prefer_id").value)
            idx = 0
            if prefer >= 0:
                matches = np.where(ids.flatten() == prefer)[0]
                if len(matches) > 0:
                    idx = int(matches[0])

            cv2.aruco.drawDetectedMarkers(img_bgr, corners_list, ids)

            marker_len = float(self.get_parameter("aruco_marker_length").value)
            corners = corners_list[idx].reshape(4, 2).astype(np.float64)

            # object points: marker frame on z=0, centered
            half = marker_len / 2.0
            objp = np.array([[-half,  half, 0],
                             [ half,  half, 0],
                             [ half, -half, 0],
                             [-half, -half, 0]], dtype=np.float64)

            K = np.array([[self.fx, 0, self.cx],
                          [0, self.fy, self.cy],
                          [0, 0, 1]], dtype=np.float64)

            flag = cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE
            ok2, rvec, tvec = cv2.solvePnP(objp, corners, K, self.dist, flags=flag)
            if not ok2:
                return None, None, "Aruco: PnP FAILED"

            R_cam_obj, _ = cv2.Rodrigues(rvec)
            t_cam_obj = tvec.reshape(3)

            poly = corners.astype(np.float32)
            return (R_cam_obj, t_cam_obj), poly, f"Aruco: FOUND id={int(ids[idx])} ({dict_name})"

        return None, None, f"Unknown detector '{det}'"

    def make_polygon_mask(self, h, w, poly_px):
        mask = np.zeros((h, w), dtype=np.uint8)
        if poly_px is None or len(poly_px) < 3:
            return mask
        poly = poly_px.reshape(-1, 1, 2).astype(np.int32)
        cv2.fillPoly(mask, [poly], 255)
        return mask

    def on_sync(self, img_msg: Image, cloud_msg: PointCloud2):
        if not self.camera_ready:
            return

        lidar_frame = self.get_parameter("lidar_frame").value
        cam_frame = self.get_parameter("camera_frame").value

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=cam_frame,
                source_frame=lidar_frame,
                time=rclpy.time.Time()  # latest
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return

        T_cam_lidar = tf_to_T(tf)

        try:
            img_bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        h, w = img_bgr.shape[:2]
        overlay = img_bgr.copy()

        # 1) detect target (optional) -> pose + polygon
        pose, poly_px, det_label = self.estimate_target_pose_and_polygon(overlay)

        # draw label
        if det_label:
            cv2.putText(overlay, det_label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 255, 0) if "FOUND" in det_label else (0, 0, 255), 2)

        mask = self.make_polygon_mask(h, w, poly_px) if pose is not None else None

        # 2) read lidar points
        max_pts = int(self.get_parameter("max_points").value)
        pts = []
        for p in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append([p[0], p[1], p[2], 1.0])
            if len(pts) >= max_pts:
                break
        if not pts:
            self.publish_image(overlay, img_msg.header)
            return

        P = np.array(pts, dtype=np.float64).T  # 4xN
        Pc = (T_cam_lidar @ P)                 # 4xN (camera frame)

        X = Pc[0, :]; Y = Pc[1, :]; Z = Pc[2, :]

        min_z = float(self.get_parameter("min_z_cam").value)
        max_z = float(self.get_parameter("max_z_cam").value)
        valid = (Z > min_z) & (Z < max_z)
        X = X[valid]; Y = Y[valid]; Z = Z[valid]
        Pcam = np.vstack([X, Y, Z]).T  # Nx3

        if len(Pcam) == 0:
            self.publish_image(overlay, img_msg.header)
            return

        # 3) project to pixels
        u = (self.fx * (X / Z) + self.cx).astype(np.int32)
        v = (self.fy * (Y / Z) + self.cy).astype(np.int32)
        in_img = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u = u[in_img]; v = v[in_img]
        Zp = Z[in_img]
        Pcam = Pcam[in_img]

        if len(Pcam) == 0:
            self.publish_image(overlay, img_msg.header)
            return

        # 4) compute plane error if target detected
        color_mode = self.get_parameter("color_mode").value.lower().strip()
        plane_stats = None
        colors = None

        if pose is not None and mask is not None:
            R_cam_obj, t_cam_obj = pose
            # plane normal in camera: n = R * [0,0,1]
            n = (R_cam_obj @ np.array([0.0, 0.0, 1.0], dtype=np.float64)).reshape(3)
            n = n / (np.linalg.norm(n) + 1e-12)

            inside = mask[v, u] > 0
            P_tgt = Pcam[inside]

            if len(P_tgt) >= 30:
                # point-to-plane distance (meters)
                d = np.abs((P_tgt - t_cam_obj.reshape(1, 3)) @ n.reshape(3, 1)).reshape(-1)
                rms = float(np.sqrt(np.mean(d*d)))
                mean = float(np.mean(d))
                mx = float(np.max(d))
                plane_stats = (len(P_tgt), mean, rms, mx)

                cv2.putText(overlay, f"PlaneErr N={len(P_tgt)} mean={mean*1000:.1f}mm rms={rms*1000:.1f}mm max={mx*1000:.1f}mm",
                            (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                if color_mode == "plane_error":
                    mm = np.clip(d * 1000.0, 0.0, 50.0)  # clamp at 50mm
                    val = (255.0 * (1.0 - (mm / 50.0))).astype(np.uint8)
                    c = cv2.applyColorMap(val.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
                    colors = np.zeros((len(Pcam), 3), dtype=np.uint8)
                    colors[:] = np.array([200, 200, 200], dtype=np.uint8)
                    colors[inside] = c
            else:
                cv2.putText(overlay, f"PlaneErr: not enough pts in target ({len(P_tgt)})",
                            (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # default coloring by depth
        if colors is None:
            z_norm = np.clip((Zp - min_z) / max(1e-6, (max_z - min_z)), 0.0, 1.0)
            z_u8 = (255.0 * (1.0 - z_norm)).astype(np.uint8)
            colors = cv2.applyColorMap(z_u8.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

        radius = int(self.get_parameter("point_radius").value)
        for (px, py, c) in zip(u, v, colors):
            cv2.circle(overlay, (int(px), int(py)), radius, (int(c[0]), int(c[1]), int(c[2])), -1)

        if plane_stats is not None:
            N, mean, rms, mx = plane_stats
            self.get_logger().info(f"[PlaneErr] N={N} mean={mean:.6f}m rms={rms:.6f}m max={mx:.6f}m")

        if bool(self.get_parameter("save_on_detect").value) and pose is not None and "FOUND" in det_label:
            out_dir = self.get_parameter("save_dir").value
            os.makedirs(out_dir, exist_ok=True)
            ts = int(time.time() * 1000)
            out_path = os.path.join(out_dir, f"overlay_{ts}.png")
            cv2.imwrite(out_path, overlay)
            self.get_logger().info(f"Saved {out_path}")

        self.publish_image(overlay, img_msg.header)

    def publish_image(self, img_bgr, header):
        out = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        out.header = header
        self.pub.publish(out)


def main():
    # 运行方式：
    #   python3 lidar_camera_overlay_v2.py          -> 仅投影叠加（detector=none）
    #   python3 lidar_camera_overlay_v2.py chess    -> 运行时强制棋盘格检测
    #   python3 lidar_camera_overlay_v2.py aruco    -> 运行时强制 aruco 检测
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument("mode", nargs="?", default="", choices=["", "chess", "aruco"],
                        help="不填=仅投影；填 chess=棋盘格；填 aruco=Aruco")
    args, ros_args = parser.parse_known_args()

    # YAML：与脚本同目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    yaml_path = os.path.join(script_dir, "lidar_overlay.yaml")
    yaml_cfg = load_yaml_config(yaml_path)

    rclpy.init(args=ros_args)
    node = LidarCameraOverlayV2(yaml_cfg=yaml_cfg, mode_override=args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

