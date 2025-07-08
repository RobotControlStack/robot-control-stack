# Base image with Python 3.10 and slim Debian system
FROM python:3.10-slim

# Avoid interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Optional: create a user (improves container security)
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
 && rm -rf /var/lib/apt/lists/*

# Switch to non-root user
USER devuser

# Set up virtual environment (Python)
RUN python3 -m venv /home/devuser/.venv
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
RUN pip install -e .

# Default command
CMD ["python3"]
