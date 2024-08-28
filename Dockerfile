FROM ros:noetic-ros-core

RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
        ffmpeg \
        libsm6 \
        libxext6 \
        ros-noetic-cv-bridge \
        python3-cv-bridge \
        python3-opencv \
        python3-tqdm \
        python3-progressbar \
        wget \
    && rm -rf /var/lib/apt/lists/*
