import numpy as np
import os
import cv2
import argparse
import shutil
import copy
import rosbag
from cv_bridge import CvBridge

def matching_time_indices(stamps_1, stamps_2,
                          max_diff = 0.01,
                          offset_2 = 0.0):
    """
    Searches for the best matching timestamps of two lists of timestamps
    and returns the list indices of the best matches.
    :param stamps_1: first vector of timestamps (numpy array)
    :param stamps_2: second vector of timestamps (numpy array)
    :param max_diff: max. allowed absolute time difference
    :param offset_2: optional time offset to be applied to stamps_2
    :return: 2 lists of the matching timestamp indices (stamps_1, stamps_2)
    """
    matching_indices_1 = []
    matching_indices_2 = []
    stamps_2 = copy.deepcopy(stamps_2)
    stamps_2 += offset_2
    for index_1, stamp_1 in enumerate(stamps_1):
        diffs = np.abs(stamps_2 - stamp_1)
        index_2 = int(np.argmin(diffs))
        if diffs[index_2] <= max_diff:
            matching_indices_1.append(index_1)
            matching_indices_2.append(index_2)
    return matching_indices_1, matching_indices_2

def main():
    """Extract a folder of images from a rosbag.
    """
    global seq_base_folder_sync
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", nargs='+', type=str, help="Image and IMU topics.",
                        default=["/camera/color/image_raw",
                                                   "/camera/aligned_depth_to_color/image_raw", "/camera/imu"])

    args = parser.parse_args()

    print "Extract images from %s on topic %s into %s" % (args.bag_file,
                                                          args.image_topic, args.output_dir)

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    color_count = 0
    depth_count = 0
    imu_count = 0
    color_stamps = {}
    depth_stamps = {}  # use depth stamps as master and color as aligner for time stamp sync
    if not os.path.exists(args.output_dir):
        os.mkdir(args.output_dir)
    f = open(os.path.join(args.output_dir, "imu.txt"), 'w')
    # you can use topics= [args.image_topic] to specify your topic name
    for topic, msg, t in bag.read_messages(topics=args.image_topic):
        cv_img = None
        if "image" in topic:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        output_fname = ""
        if topic == "/camera/color/image_raw":
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            color_directory_path = os.path.join(args.output_dir, "color")
            if not os.path.exists(color_directory_path):
                os.makedirs(color_directory_path)
            output_fname = os.path.join(color_directory_path, "%06i.png" % color_count)
            print "Wrote color image %i" % color_count
            color_stamps[msg.header.stamp.to_sec()] = output_fname
            color_count += 1
        if topic == "/camera/aligned_depth_to_color/image_raw":
            depth_directory_path = os.path.join(args.output_dir, "depth")
            if not os.path.exists(depth_directory_path):
                os.makedirs(depth_directory_path)
            output_fname = os.path.join(depth_directory_path, "%06i.png" % depth_count)
            print "Wrote depth image %i" % depth_count
            depth_stamps[msg.header.stamp.to_sec()] = output_fname
            depth_count += 1
        if topic == "/camera/imu":
            f.write('%.9f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                    (msg.header.stamp.to_sec(),
                     msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                     msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        if "image" in topic:
            cv2.imwrite(output_fname, cv_img)
    bag.close()

    # start the time sync:
    depth_stamps_t = np.fromiter(depth_stamps.iterkeys(), dtype=float)
    color_stamps_t = np.fromiter(color_stamps.iterkeys(), dtype=float)
    depth_stamps_t.sort()
    color_stamps_t.sort()
    # find the matching indices between depth and color time stamps
    matching_indices_1, matching_indices_2 = matching_time_indices(depth_stamps_t, color_stamps_t)
    # len(matching_indices_1) == len(matching_indices_2)
    matched_depth_stamps_t = []
    for depth_iter_idx, dpt_index in enumerate(matching_indices_1):
        # obtain the file path of the matched indices
        depth_file_path = depth_stamps[depth_stamps_t[dpt_index]]
        color_file_path = color_stamps[color_stamps_t[matching_indices_2[depth_iter_idx]]]
        depth_file_name = depth_file_path.split("/")[-1]
        color_file_name = color_file_path.split("/")[-1]
        seq_base_folder = os.path.abspath(depth_file_path + "/../../")
        seq_base_folder_sync = seq_base_folder+"_sync"
        seq_depth_folder_sync = os.path.join(seq_base_folder_sync, "depth")
        seq_color_folder_sync = os.path.join(seq_base_folder_sync, "color")
        if not os.path.exists(seq_base_folder_sync):
            os.makedirs(seq_base_folder_sync)
            os.makedirs(seq_depth_folder_sync)
            os.makedirs(seq_color_folder_sync)
        # save the depth and color sync files to the new folder with new indices
        shutil.copy(depth_file_path, os.path.join(seq_depth_folder_sync, "%06i.png" % depth_iter_idx))
        shutil.copy(color_file_path, os.path.join(seq_color_folder_sync, "%06i.png" % depth_iter_idx))
        # record the time stamp
        matched_depth_stamps_t.append([depth_iter_idx, depth_stamps_t[dpt_index]])
    matched_depth_stamps_t = np.array(matched_depth_stamps_t)
    # copy imu.txt
    shutil.copy(os.path.join(args.output_dir, "imu.txt"), os.path.join(seq_base_folder_sync, "imu.txt"))
    # save
    np.savetxt(os.path.join(seq_base_folder_sync, "timestamps.txt"), matched_depth_stamps_t, fmt=('%06d', '%.9f'))
    return

if __name__ == '__main__':
    main()
