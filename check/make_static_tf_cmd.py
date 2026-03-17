import json
import numpy as np


    ######################################################
    #用法：终端输入 python make_static_tf_cmd.py calib.json #
    ######################################################


def quat_to_R(q):
    x, y, z, w = q
    # normalized just in case
    n = (x*x + y*y + z*z + w*w) ** 0.5
    x, y, z, w = x/n, y/n, z/n, w/n
    R = np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)],
    ])
    return R

def R_to_quat(R):
    # returns (x,y,z,w)
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    else:
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif i == 1:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
    # normalize
    q = np.array([x,y,z,w])
    q = q / np.linalg.norm(q)
    return q

def main(path, parent="velodyne", child="camera_color_optical_frame"):
    data = json.load(open(path, "r"))
    arr = data["results"]["T_lidar_camera"]
    t = np.array(arr[:3], dtype=float)
    q = np.array(arr[3:7], dtype=float)  # x,y,z,w

    print("== lidar -> camera (as stored in calib.json) ==")
    print(f"ros2 run tf2_ros static_transform_publisher "
          f"--x {t[0]} --y {t[1]} --z {t[2]} "
          f"--qx {q[0]} --qy {q[1]} --qz {q[2]} --qw {q[3]} "
          f"--frame-id {parent} --child-frame-id {child}")

    # inverse: camera -> lidar
    R = quat_to_R(q)
    R_inv = R.T
    t_inv = -R_inv @ t
    q_inv = R_to_quat(R_inv)

    print("\n== camera -> lidar (inverse) ==")
    print(f"ros2 run tf2_ros static_transform_publisher "
          f"--x {t_inv[0]} --y {t_inv[1]} --z {t_inv[2]} "
          f"--qx {q_inv[0]} --qy {q_inv[1]} --qz {q_inv[2]} --qw {q_inv[3]} "
          f"--frame-id {child} --child-frame-id {parent}")

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("calib_json")
    ap.add_argument("--parent", default="livox_frame")
    ap.add_argument("--child", default="camera_color_optical_frame")
    args = ap.parse_args()
    main(args.calib_json, args.parent, args.child)
