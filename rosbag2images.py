#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology
# Copyright 2024 CyberAgent AI Lab
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
# LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# This file is a modification of the work by priteshgohil(github.com/priteshgohil).
# The original file is at https://gist.github.com/priteshgohil/c3cf492b5705cd5536a68b60a0e89c52
# and distributed under the MIT license.

"""
    Extract images from a rosbag.
    How to use: In terminal, cd DIRECTORY_OF_THIS_FILE and then type following
                python bag_to_images.py --bag_file camera_odom_compressed.bag --output_dir output/ --image_topic '/camera/image_raw'
                python bag_to_images.py --bag_file my_rosbag_file.bag --output_dir output/ --image_topic '/eGolf/front_cam/image_raw'

"""

import os
import argparse
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from tqdm import tqdm


description = "Extraction in progress"


def image_index_to_path(output_dir, index):
    return os.path.join(output_dir, "{:06}.png".format(index))


def extract_and_save_image(messages, output_dir, n_frames, swaprb):
    bridge = CvBridge()
    for i, (_, msg, t) in tqdm(enumerate(messages), total=n_frames, desc=description):
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if swaprb:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        path = image_index_to_path(output_dir, i)
        cv2.imwrite(path, image)


def extract_and_save_compressed_image(messages, output_dir, n_frames, swaprb):
    bridge = CvBridge()
    for i, (_, msg, t) in tqdm(enumerate(messages), total=n_frames, desc=description):
        image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if swaprb:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        path = image_index_to_path(output_dir, i)
        cv2.imwrite(path, image)


def main():
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("--bag_file", help="Input ROS bag.")
    parser.add_argument("--output_dir", help="Output directory.")
    parser.add_argument("--image_topic", help="Single image topic or list of topics")
    parser.add_argument(
        "--swaprb", help="Swap red and blue channel", action="store_true"
    )

    args = parser.parse_args()

    bag = rosbag.Bag(args.bag_file, "r")

    topics = bag.get_type_and_topic_info().topics

    if args.image_topic not in topics:
        print(f"Topic '{args.image_topic}' does not exist in rosbag.")
        bag.close()
        return

    msg_type = topics[args.image_topic].msg_type
    is_compressed_image_type = msg_type == "sensor_msgs/CompressedImage"
    is_image_type = msg_type == "sensor_msgs/Image"

    if (not is_image_type) and (not is_compressed_image_type):
        print(
            f"Message type must be either of 'sensor_msgs/Image' or "
            f"'sensor_msgs/CompressedImage'. Actual: '{msg_type}'"
        )
        bag.close()
        return

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    n_frames = bag.get_message_count(args.image_topic)
    messages = bag.read_messages(topics=[args.image_topic])

    if is_image_type:
        extract_and_save_image(messages, args.output_dir, n_frames, args.swaprb)

    if is_compressed_image_type:
        extract_and_save_compressed_image(
            messages, args.output_dir, n_frames, args.swaprb
        )

    bag.close()


main()
