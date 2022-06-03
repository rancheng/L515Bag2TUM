import os
import glob
import numpy as np
import argparse
import cv2
from bag2dataset import matching_time_indices
import math
import shutil
import json

_EPS = np.finfo(float).eps * 4.0
AABB_SCALE = 4.0
def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.
    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= np.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)


def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()

def parse_pose_graph(pose_graph_fname):
    with open(pose_graph_fname, "r") as pose_graph_f:
        pose_graph_data = pose_graph_f.readlines()
        pose_tf_list = []
        pose_ts_list = []
        for pg_item in pose_graph_data:
            tmp_data = pg_item.rstrip(",\n").split(",")
            tmp_ts = int(tmp_data[0])/1e9
            tx = float(tmp_data[1])
            ty = float(tmp_data[2])
            tz = float(tmp_data[3])
            qw = float(tmp_data[4])
            qx = float(tmp_data[5])
            qy = float(tmp_data[6])
            qz = float(tmp_data[7])
            trans_mat = quaternion_matrix((qw, qx, qy, qz))
            trans_mat[0, -1] = tx
            trans_mat[1, -1] = ty
            trans_mat[2, -1] = tz
            pose_tf_list.append(trans_mat)
            pose_ts_list.append(tmp_ts)
    return pose_ts_list, pose_tf_list

def sharpness(imagePath):
    image = cv2.imread(imagePath)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fm = variance_of_laplacian(gray)
    return fm


def dataset2nerf(dataset_dir):
    out_dataset_dir = dataset_dir + "_out"
    out_img_dir = os.path.join(out_dataset_dir, "images")
    times_fname = os.path.join(dataset_dir, "timestamps.txt")
    pose_fname = os.path.join(dataset_dir, "vins_result_loop.csv")
    times_data = np.genfromtxt(times_fname)
    pose_ts, pose_data = parse_pose_graph(pose_fname)
    pose_ts_idx, pose_idx_in_times_idx = matching_time_indices(pose_ts, times_data[:, 1])
    w = 1280
    h = 720
    fl_x = 906.259765625
    fl_y = 906.550964355469
    cx = 642.896362304688
    cy = 353.142242431641
    k1 = 0.163392618298531
    k2 = -0.507503688335419
    p1 = -0.00113613810390234
    p2 = -0.00013459162437357
    angle_x = math.atan(w / (fl_x * 2)) * 2
    angle_y = math.atan(h / (fl_y * 2)) * 2
    fovx = angle_x * 180 / math.pi
    fovy = angle_y * 180 / math.pi
    out = {
        "camera_angle_x": angle_x,
        "camera_angle_y": angle_y,
        "fl_x": fl_x,
        "fl_y": fl_y,
        "k1": k1,
        "k2": k2,
        "p1": p1,
        "p2": p2,
        "cx": cx,
        "cy": cy,
        "w": w,
        "h": h,
        "aabb_scale": AABB_SCALE,
        "frames": [],
    }
    if not os.path.exists(out_dataset_dir):
        os.makedirs(out_dataset_dir)
        if not os.path.exists(out_img_dir):
            os.makedirs(out_img_dir)
            print "converting..."
            for pose_idx in pose_ts_idx:
                print "img: %d" % pose_idx
                flip_mat = np.array([
                    [1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]
                ])
                pose_tf_data = pose_data[pose_idx]  # VINS coordinate system is same as COLMAP ones
                pose_tf_data_flipped = np.matmul(pose_tf_data, flip_mat)
                img_fidx = "%06d" % times_data[pose_idx_in_times_idx[pose_idx]][0]
                file_path = "images/" + img_fidx + ".png"
                img_fname = os.path.join(dataset_dir, "color", img_fidx+".png")
                img_out_fname = os.path.join(out_img_dir, img_fidx+".png")
                shutil.copy(img_fname, img_out_fname)
                sharpness_i = sharpness(img_fname)
                frame = {
                    "file_path": file_path,
                    "sharpness": sharpness_i,
                    "transform_matrix": pose_tf_data.tolist()
                }
                out['frames'].append(frame)
    with open(os.path.join(out_dataset_dir, "transforms.json"), "w") as outfile:
        json.dump(out, outfile, indent=2)


def main():
    """convert the dataset to nerf script
    """
    global seq_base_folder_sync
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("dataset_dir", help="dataset directory.")
    args = parser.parse_args()
    dataset2nerf(args.dataset_dir)


if __name__ == '__main__':
    main()
