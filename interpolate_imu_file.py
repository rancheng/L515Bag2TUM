# BSD 3-Clause License
#
# This file is part of the DM-VIO-Python-Tools.
# https://github.com/lukasvst/dm-vio-python-tools
#
# Copyright (c) 2022, Lukas von Stumberg, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
# disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
# following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
# products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# python interpolate_imu_file.py --input "imu_origin.txt" --times "cam0/times_nesc.txt" --output imu.txt

import argparse
import numpy as np


def interpolate_imu_file(imu_input_filename, times_input_filename, imu_output_filename):
    """Inserts interpolated IMU measurements at all timestamps of images."""
    imu_data = np.loadtxt(imu_input_filename,  usecols=(1,2,3,4,5,6))
    imu_data_times = np.loadtxt(imu_input_filename, int,  usecols=(0))
    times_data = np.loadtxt(times_input_filename, dtype=int)
    # times_data = np.load(times_input_filename) # load npy format

    image_times = times_data[:]
    imu_times = imu_data_times
    min_imu_time = imu_times[0]
    max_imu_time = imu_times[imu_times.shape[0] - 1]

    filtered_times = image_times[np.logical_and(image_times <= max_imu_time, image_times >= min_imu_time)]

    all_times = np.concatenate((filtered_times, imu_times), axis=0)
    all_times.sort()

    interpolated = [np.interp(all_times, imu_times, imu_data[:, i]) for i in range(6)]
    interpolated.insert(0, all_times)
    result = np.ndarray((7, len(all_times)), dtype=object)
    result[0] = all_times
    result[1:] = interpolated[1:]
    interpolated_stacked = result.transpose()
    np.savetxt(imu_output_filename, interpolated_stacked, fmt=['%1i'] + 6 * ['%1f'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Interpolate IMU data to have a 'fake' measurement at each time in the times file.")
    parser.add_argument('--input', type=str, default=None, help="Path to IMU input file.")
    parser.add_argument('--times', type=str, default=None, help="Path to input times file.")
    parser.add_argument('--output', type=str, default=None, help="Path to output file.")
    args = parser.parse_args()
    interpolate_imu_file(args.input, args.times, args.output)
