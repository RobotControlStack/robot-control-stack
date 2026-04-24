FROM stereolabs/zed:5.2-gl-devel-cuda12.8-ubuntu24.04
# FROM stereolabs/zed:5.0-gl-devel-cuda12.1-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    UV_LINK_MODE=copy

WORKDIR /opt/rcs-src

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    libgl1 \
    libglib2.0-0 \
    libglfw3-dev \
    libpoco-dev \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

RUN curl -LsSf https://astral.sh/uv/install.sh | sh

ENV PATH="/opt/venv/bin:/root/.local/bin:${PATH}" \
    VIRTUAL_ENV=/opt/venv

RUN uv python install 3.11 \
    && uv venv --python 3.11 /opt/venv \
    && python -m ensurepip --upgrade

# Install the ZED Python bindings into the same virtualenv used by the project.
RUN uv pip install requests \
    && python /usr/local/zed/get_python_api.py

COPY . /opt/rcs-src
COPY docker/link-editable-source.sh /usr/local/bin/link-editable-source

RUN chmod +x /usr/local/bin/link-editable-source \
    && uv pip install \
    build \
    wheel \
    "setuptools>=45" \
    "scikit-build-core>=0.3.3" \
    pybind11 \
    cmake \
    ninja \
    "mujoco==3.2.6" \
    "pin==3.7.0" \
    && uv pip install /opt/rcs-src \
    && uv pip install --no-build-isolation /opt/rcs-src/extensions/rcs_fr3 \
    && uv pip install /opt/rcs-src/extensions/rcs_realsense \
    && uv pip install /opt/rcs-src/extensions/rcs_robotiq2f85 \
    && uv pip install /opt/rcs-src/extensions/rcs_zed

WORKDIR /workspace/robot-control-stack

CMD ["sh"]
