#!/bin/bash

if [ "$#" -ne 1 ] && [ "$#" -ne 3 ] ; then
    echo "Usage:"
    echo ""
    echo "To list topics, run "
    echo "  $ ./extract.sh <path to rosbag file>"
    echo ""
    echo "To extract images from a rosbag file, run "
    echo "  $ ./extract.sh <path to rosbag file> <path to output image directory> <topic name>"
    exit
fi

bagfile=$(readlink -f $1)

if [ ! -f $bagfile ]; then
    echo "$bagfile is not a file."
    exit
fi

if [ "$#" -eq 1 ]; then
    docker run --rm \
        -v $bagfile:/bagfile \
        rosbag2images \
        rosbag info /bagfile
    exit
fi

if [ ! -d $2 ]; then
    echo "Directory $2 does not exist. Trying to create it."
    mkdir -p $2 && echo "Directory $2 is created."
fi

imagedir=$(readlink -f $2)

docker run --rm \
    -v $bagfile:/bagfile \
    -v $imagedir:/imagedir \
    -v $(pwd)/rosbag2images.py:/rosbag2images.py \
    rosbag2images \
    python3 rosbag2images.py --bag_file /bagfile --output_dir /imagedir --image_topic $3
