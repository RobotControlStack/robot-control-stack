# Base image with Python 3.10 and slim Debian system
FROM python:3.10-slim

# System configuration
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    CMAKE_BUILD_PARALLEL_LEVEL=2 \
    SKBUILD_BUILD_OPTIONS="-j2" \
    MAKEFLAGS="-j2"
#############################################################################
# Explanation of environment variables:    
# DEBIAN_FRONTEND=noninteractive: Disables interactive prompts during apt-get install.
# PYTHONDONTWRITEBYTECODE=1: Prevents .pyc files, keeping the image clean.
# PYTHONUNBUFFERED=1: Ensures logs are written directly (no buffering).
# Parallel build settings for CMake, scikit-build, and make to improve build speed.
#############################################################################

# create a user to avoid running as root
RUN useradd -ms /bin/bash devuser
WORKDIR /home/devuser
USER root

# Install system dependencies (from debian_deps.txt manually inlined here)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    libxslt-dev \
    libcoin-dev \
    libccd-dev \
    libglfw3-dev \
    libboost-all-dev \
    liblzma-dev \
    libxml2-dev \
    libxslt1-dev \
    ninja-build \
    clang \
    clang-format \
    clang-tidy \
    pkg-config \
    curl \
    unzip \
    wget \
    libgl1 \
    patchelf \
    python3-venv \
    libegl-dev \
    libegl1-mesa-dev \ 
    libglib2.0-dev \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Remove root password so `su -` works from devuser
RUN passwd -d root


# Switch to non-root user
USER devuser

# Set up virtual environment (Python)
RUN python3 -m venv /home/devuser/.venv
# prepend /home/devuser/.venv/bin to the existing PATH
# This ensures that the virtual environment's Python and pip are used by default.
ENV PATH="/home/devuser/.venv/bin:$PATH"

# Copy project files into container
COPY --chown=devuser . /home/devuser/project
WORKDIR /home/devuser/project

# Upgrade pip and install project build tools
RUN pip install --upgrade pip setuptools
RUN pip config --site set global.no-build-isolation false

# Install development dependencies
RUN pip install -r requirements_dev.txt

# Install the package in editable mode (CMake + pybind11 + scikit-build-core triggered)
RUN pip install -e . --no-cache-dir --verbose --no-build-isolation

# Default command that runs when you start a container without specifying a command explicitly.
CMD ["python3"]

######################################################################
# Build the Docker image with specified memory limits
# To build the Docker image, run the following command in the terminal:
# docker build --memory=4g --memory-swap=6g . -t rcs-dev
######################################################################
# --memory=4g          Limit the build process to 4 GB of RAM
# --memory-swap=6g     Limit total memory (RAM + swap) to 6 GB
# .                    Use current directory as the Docker context
# -t rcs-dev           Tag the built image as "rcs-dev"
######################################################################

######################################################################
# Run the Docker container interactively (without GUI)
# docker run -it --rm rcs-dev bash
######################################################################
# -it                  Interactive mode with TTY
# --rm                 Automatically remove container after exit
# rcs-dev              Name of the Docker image to run
# bash                 Start an interactive bash shell inside the container
######################################################################

######################################################################
# Optional: Run GUI applications from inside the container
# First, allow X11 connections from Docker containers:
# Run this command on the host machine:
# xhost +local:docker
######################################################################
# xhost                A utility to manage X11 display access control
# +local:docker        Grant X11 access to Docker containers running locally
######################################################################

######################################################################
# Run container with GUI support (no GPU)
# docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --shm-size=1g rcs-dev bash
######################################################################
# -e DISPLAY=$DISPLAY                  Pass display info to container
# -v /tmp/.X11-unix:/tmp/.X11-unix    Mount X11 socket for GUI apps
# --shm-size=1g                        Increase shared memory for rendering (useful for tools like MuJoCo)
######################################################################

######################################################################
# Run container with NVIDIA GPU support
# Make sure NVIDIA Container Toolkit is installed and configured
# For more info: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
# docker run -it --rm --gpus all --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --shm-size=1g rcs-dev bash
######################################################################
# --gpus all                         Enable all available GPUs
# --runtime=nvidia                   Use NVIDIA runtime for GPU access
# -e NVIDIA_VISIBLE_DEVICES=all     Expose all GPUs inside container
# -e NVIDIA_DRIVER_CAPABILITIES=all Enable all GPU features (e.g., graphics, compute)
# Other flags same as GUI setup above
######################################################################
# Run the container with NVIDIA GPU support and hardware access:
# docker run -it --rm --gpus all --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --shm-size=2g --network host --privileged --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit rttime=-1 --ulimit memlock=8428281856 -v /dev:/dev rcs-dev bash
# Optional flags for running the container with hardware access:
#   --network host \                             # Use the host's network stack (needed for low-latency ROS comms)
#   --privileged \                               # Grant full device and kernel access (required for hardware control)
#   --cap-add=SYS_NICE \                         # Allow processes to raise their scheduling priority
#   --ulimit rtprio=99 \                         # Enable real-time priority up to 99
#   --ulimit rttime=-1 \                         # Disable CPU time limit for real-time threads
#   --ulimit memlock=8428281856 \                # Lock ~8GB of RAM to prevent memory swapping
#   -v /dev:/dev \                               # Mount all host devices for hardware access (e.g., Franka arm)  
