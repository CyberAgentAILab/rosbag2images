#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This file is a modification of the work by
# priteshgohil(github.com/priteshgohil).
# The original file is at
# https://gist.github.com/priteshgohil/c3cf492b5705cd5536a68b60a0e89c52
# and distributed under the MIT license.

# Copyright 2016 Massachusetts Institute of Technology
# Tutorial : http://wiki.ros.org/rosbag/Code%20API#Python_API
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


def main():
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("--bag_file", help="Input ROS bag.")
    parser.add_argument("--output_dir", help="Output directory.")
    parser.add_argument("--image_topic", help="single image topic or list of topics")

    args = parser.parse_args()

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()

    n_frames = bag.get_message_count(args.image_topic)
    messages = bag.read_messages(topics=[args.image_topic])
    topics = bag.get_type_and_topic_info().topics

    msg_type = topics[args.image_topic].msg_type

    if args.image_topic not in topics:
        bag.close()
        print(f"Topic '{args.image_topic}' does not exist in rosbag.")
        return

    if msg_type != "sensor_msgs/Image":
        bag.close()
        print(f"Message type must be 'sensor_msgs/Image'. Actual: '{msg_type}'")
        return

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    for i, (_, msg, t) in tqdm(enumerate(messages), total=n_frames, desc=description):
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        p = os.path.join(args.output_dir, "{:06}.png".format(i))
        cv2.imwrite(p, image)

    bag.close()

main()
