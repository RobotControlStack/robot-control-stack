# Chose the base image from https://hub.docker.com/
FROM nvidia/cuda:13.1.2-cudnn-devel-ubuntu24.04

# Set variables
ARG USERNAME=vscode
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# Avoid interactive dialog
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Create new user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# For example, install python 3.12, git and curl
RUN apt-get install -y tzdata
RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt update && apt install -y python3.11 python3.11-venv python3.11-dev curl liblz4-dev git libgl1-mesa-dev cmake build-essential 
RUN apt install -y ffmpeg ca-certificates cmake curl git libgl1 libglib2.0-0 libglfw3-dev libpoco-dev ninja-build liburdfdom-dev

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
WORKDIR /workspaces

# Install python dependencies and renderpy
COPY . /workspaces
RUN python3.11 -m venv /home/$USERNAME/venv
RUN /home/$USERNAME/venv/bin/pip install --upgrade pip
RUN /home/$USERNAME/venv/bin/pip install --upgrade setuptools wheel ninja setuptools>=45 scikit-build-core>=0.3.3 mujoco>=3.3.5 pin==3.7.0 pybind11 cmake
RUN /home/$USERNAME/venv/bin/pip install --no-build-isolation --no-cache-dir -ve .
RUN /home/$USERNAME/venv/bin/pip install --no-build-isolation --no-cache-dir -e extensions/rcs_sb3
RUN /home/$USERNAME/venv/bin/pip install --no-build-isolation --no-cache-dir -e extensions/rcs_taxim
RUN cd norm2tex && /home/$USERNAME/venv/bin/pip install --no-build-isolation --no-cache-dir -e .