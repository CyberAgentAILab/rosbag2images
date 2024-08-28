## Docker environment for image extraction from ROS1 rosbag

This is a tool to extract images from a ROS1 rosbag without requiring manual setup of the ROS1 environment on the Host OS.  

## Requriement

Host OS with Docker and Bash installed.  

## Setup

First you need to run setup.sh to build the docker image.  

```
$ bash setup.sh
```

## Run

### List topics

```
$ ./extract.sh <path to rosbag file>
```

### Extract images

```
$ ./extract.sh <path to ros1 rosbag file> <path to output image directory> <topic name>
```

## Thanks

The image extraction script `rosbag2images.py` is the modification of [priteshgohil's work](https://gist.github.com/priteshgohil/c3cf492b5705cd5536a68b60a0e89c52), which is [distributed under the MIT license](https://gist.github.com/priteshgohil/c3cf492b5705cd5536a68b60a0e89c52?permalink_comment_id=5124226#gistcomment-5124226).
