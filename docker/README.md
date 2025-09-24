# Docker (GUI + GPU + HW add-ons)

**Prereqs:** Docker + docker-compose, X11 on host, NVIDIA driver + NVIDIA Container Toolkit (legacy `runtime: nvidia`).  
**Layout:** `docker/Dockerfile`, overrides in `docker/compose/` (`base.yml`, `gui.yml`, `gpu.yml`, `hw.yml`).

## Build the image
`docker-compose -f docker/compose/base.yml build dev`

## (GUI) allow X access (host)
`export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}`  
`xhost +local:docker`

## Run container with GUI + GPU + HW and open a shell
`docker-compose -f docker/compose/base.yml -f docker/compose/gui.yml -f docker/compose/gpu.yml -f docker/compose/hw.yml run --rm run bash`  
*(Use fewer `-f` files for lighter setups, e.g., GUI+GPU without HW.)*

## Inside the container
`pip install -ve extensions/rcs_fr3`  
`cd examples`  
`python fr3_env_cartesian_control.py`

## Troubleshooting
- **`nvidia-smi` missing in container:** ensure it exists on host at `/usr/bin/nvidia-smi` (GPU override bind-mounts it).  
- **GUI can’t open:** re-run the `xhost` command and confirm `$DISPLAY` is set on the host.  