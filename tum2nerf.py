import os
import glob
import numpy as np
import argparse
import cv2


def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()


def sharpness(imagePath):
    image = cv2.imread(imagePath)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fm = variance_of_laplacian(gray)
    return fm


def dataset2nerf(dataset_dir):
    out_dataset_dir = dataset_dir + "_nerf"


def main():
    """convert the dataset to nerf script
    """
    global seq_base_folder_sync
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("dataset_dir", help="dataset directory.")
    args = parser.parse_args()


if __name__ == '__main__':
    main()
