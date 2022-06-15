# -*- coding=utf-8 -*-
# rosbag转换到dm-vio数据集
# python bag2dataset.py **.bag ./

import numpy as np
import os
import cv2
import argparse
import shutil
import copy
import rosbag
from cv_bridge import CvBridge
import json

def main():
    """Extract a folder of images from a rosbag.
    """
    global seq_base_folder_sync
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")

    args = parser.parse_args()

    print("Extract images from {} into {}".format(args.bag_file, args.output_dir))

    bag = rosbag.Bag(args.bag_file, "r")

    bridge = CvBridge()

    if not os.path.exists(args.output_dir):
        os.mkdir(args.output_dir)
    f = open(os.path.join(args.output_dir, "imu_origin.txt"), 'w')
    f.write("# timestamp[ns] w.x w.y w.z a.x a.y a.z\n")

    time_stamp = []
    time_stamp_float = []
    exposure_time = {}

    color_directory_path = os.path.join(args.output_dir, "cam0/images")
    if not os.path.exists(color_directory_path):
        os.makedirs(color_directory_path)

    # you can use topics= [args.image_topic] to specify your topic name
    for topic, msg, t in bag.read_messages(topics=["/camera/infra1/image_rect_raw", "/camera/imu", "/camera/infra1/metadata"]):
        output_fname = ""
        # if msg.header.stamp.to_sec() > 1655258134.21: # uncomment this if you want to filter by timestamps
        #     continue
        if topic == "/camera/infra1/image_rect_raw":
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            cur_time = msg.header.stamp.to_nsec()
            cur_time_sec = msg.header.stamp.to_sec()
            time_stamp.append(cur_time)
            time_stamp_float.append(cur_time_sec)
            output_fname = os.path.join(color_directory_path, str(cur_time)+".jpg")
            cv2.imwrite(output_fname, cv_img)

        if topic == "/camera/imu":
            f.write('%d %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                    (msg.header.stamp.to_nsec(),
                     msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                     msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))

        if topic == "/camera/infra1/metadata":
            metadata = json.loads(msg.json_data)
            exposure_time[msg.header.stamp.to_nsec()] = metadata["actual_exposure"] * 1e-3

    bag.close()

    # np.save("timestamp.npy", np.asarray(time_stamp))
    # align the sequence length
    # time_stamp = time_stamp[:len(exposure_time.keys())]
    # time_stamp_float = time_stamp_float[:len(exposure_time.keys())]
    time_result = np.ndarray((len(time_stamp), 3), dtype=object)
    time_stamp = np.array(time_stamp)
    time_stamp_float = np.array(time_stamp_float)
    time_result[:, 0] = time_stamp
    time_result[:, 1] = time_stamp_float
    exp_time_klist = list(exposure_time.keys())
    mask_list = []
    for i in range(len(time_stamp)):
        if time_stamp[i] in exp_time_klist:
            time_result[i, 2] = exposure_time[time_stamp[i]]
            mask_list.append(i)
        else:
            print("no exp time found, skipped: {}".format(i))
    time_result = time_result[mask_list, :]
    time_stamp = time_stamp[mask_list]
    time_stamp_float = time_stamp_float[mask_list]
    np.savetxt(os.path.join(args.output_dir, "cam0/times.txt"), time_result, fmt=['%1i'] + ['%1f'] + ['%1f'])
    np.savetxt(os.path.join(args.output_dir, "cam0/times_nesc.txt"), time_stamp, fmt=['%1i'])
    return

if __name__ == '__main__':
    main()
