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
# Build the Docker image with the specified memory limits
# To build the Docker image, run the following command in the terminal:
# docker build --memory=4g --memory-swap=6g . -t rcs-dev
######################################################################
# --memory=4g	Limit build process to 4 GB RAM
# --memory-swap=6g	Limit RAM + swap to 6 GB total
# .	Use current directory for Docker context
# -t rcs-dev	Tag the image as rcs-dev
######################################################################
# Run the Docker container interactively 
# docker run -it --rm rcs-dev bash
######################################################################
# Optional: for GUI applications, you might need to set up X11 forwarding
# Example command to run the container with GUI support
# Note: Ensure your host system allows X11 connections from the container
#  run the following command in the terminal:
# xhost +local:docker
######################################################################
# Temporarily allows Docker containers (which run as separate users) to connect to your host's X11 display server and open GUI windows.
# xhost: A Linux tool for managing access to the X11 display server.
# +local:docker: Grants access
######################################################################
# with no GPU support  
# This command runs the container with the current user's display settings, allowing GUI applications to render on your host machine.
# Make sure your host's X11 server is configured to allow connections from the Docker container.
######################################################################
# docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --shm-size=1g rcs-dev  bash
######################################################################
# Explanation of the command:
# docker run	Starts a new Docker container.
# -it	Interactive + TTY: Keeps the terminal session open and usable.
# --rm	Automatically removes the container after it exits (no leftovers).
# -e DISPLAY=$DISPLAY	Passes your host’s display address (usually :0) into the container, so it knows where to draw windows.
# -v /tmp/.X11-unix:/tmp/.X11-unix	Mounts the Unix socket for X11 communication from host to container — this is how the GUI actually connects.
# --shm-size=1g	Enlarges the container’s shared memory space (default is too small for MuJoCo rendering).
# rcs-dev	The name of your Docker image to run — built using your Dockerfile.
# bash	Starts a bash shell inside the container, allowing you to run commands interactively.
######################################################################
# If you want to run the container with NVIDIA GPU support, you can use the following command:
# Make sure you have the NVIDIA Container Toolkit installed and configured.
# This allows Docker to utilize the GPU resources of your host machine.
# The command below assumes you have the NVIDIA runtime set up correctly.
######################################################################
# docker run -it --rm --gpus all --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --shm-size=1g rcs-dev bash
######################################################################
# Explanation of the command:
# --gpus all	Allows the container to access all available GPUs on the host.
# --runtime=nvidia	Specifies the NVIDIA runtime for Docker, enabling GPU support.
# -e NVIDIA_VISIBLE_DEVICES=all	Exposes all GPUs to the container.
# -e NVIDIA_DRIVER_CAPABILITIES=all	Allows the container to use all NVIDIA driver capabilities, such as compute and graphics.
# --shm-size=1g increases the shared memory size to 1 GB, which is often necessary for applications that require more memory for rendering or processing.


