# Docker

Build the image from the repository root:

```sh
docker build -f docker/Dockerfile -t rcs-dev .
```

Run the development container with Docker Compose:

```sh
xhost +si:localuser:root
docker compose -f docker/compose/dev.yml run --rm rcs
```

Notes:

- The compose setup bind-mounts the repository into `/workspace/robot-control-stack`.
- The compose service is tagged as `rcs-dev`, so the manual Docker build tag and the Compose service refer to the same image name.
- The Docker image installs the ZED Python API (`pyzed`) during build by running `/usr/local/zed/get_python_api.py` inside the project virtualenv.
- The compose setup requests GPU access using a device reservation, which is more widely supported than the newer service-level `gpus:` key.
- The host should grant local X11 access before starting the container: `xhost +si:localuser:root`.
- `~/zed_models` is mounted into `/usr/local/zed/resources` to match the direct `docker run` setup.
- `/dev/dri` is masked inside the container so host Mesa/AMD render nodes do not override the NVIDIA runtime devices.
- NVIDIA PRIME/GLX environment variables are exported to bias OpenGL/EGL selection toward the NVIDIA stack when using X11 forwarding.
- Python source changes are picked up from the mounted repo, including `extensions/rcs_zed`.
- If you change C++ code in `rcs` or `rcs_fr3`, rebuild the image.
- For non-GPU hosts, comment out the GPU-related lines in `docker/compose/dev.yml`.
