# Docker
This setup lets you **build once** and then add capabilities as overrides — GUI, GPU, and Hardware (HW) — onto a single runtime service **without touching your main repo files**.

---

## Prerequisites

- **Docker** + **docker-compose** (either v1 or v2 plugin)
- **NVIDIA drivers** on host (`nvidia-smi` works on host)
- **NVIDIA Container Toolkit** installed on host (legacy runtime: `nvidia`)
- **X11** on host (Linux)  
---

## Layout

    docker/
      Dockerfile             # Your build definition
      compose/
        base.yml   # dev (build + source mount) and run (no mount)
        gui.yml    # adds DISPLAY, XAUTHORITY, X11 socket
        gpu.yml    # NVIDIA runtime + envs, mounts nvidia-smi
        hw.yml     # /dev, caps, ulimits (heavy hardware access)

---

## What Each File Does

- **base.yml**  
  - `dev`: builds the image from `docker/Dockerfile` and mounts your source  
  - `run`: clean runtime service (no source mount)
- **gui.yml** — Adds X11 env/socket so GUI apps can display on host
- **gpu.yml** — Enables GPU via legacy runtime: `nvidia` (for older compose) and ensures `nvidia-smi` is available in the container
- **hw.yml** — Grants broad hardware access

---

## How to Use

From the repo root:

### 1) Build the image
Uses `docker/Dockerfile`. Only `dev` mounts your source.

    docker-compose -f docker/compose/base.yml build dev

### 2) (For GUI) Allow root to use your X server

    xhost +local:docker

### 3) Run with different capability combinations

**GUI + GPU**

    docker-compose \
      -f docker/compose/base.yml \
      -f docker/compose/gui.yml \
      -f docker/compose/gpu.yml \
      run --rm run bash

**GUI + GPU + HW**

    docker-compose \
      -f docker/compose/base.yml \
      -f docker/compose/gui.yml \
      -f docker/compose/gpu.yml \
      -f docker/compose/hw.yml \
      run --rm run bash

---

## Quick Checks Inside the Container

Verify GPU:

    nvidia-smi

Test GUI apps:

    apt-get update && apt-get install -y x11-apps && xclock &

---

## Sanity Check Before Running

See the fully merged service to confirm all env/volumes are present:

    docker-compose \
      -f docker/compose/base.yml \
      -f docker/compose/gui.yml \
      -f docker/compose/gpu.yml \
      -f docker/compose/hw.yml \
      config | sed -n '/run:/,/^[^ ]/p'

---

## Example: FR3 Environment in Docker

1) Build the image:

       docker-compose -f docker/compose/base.yml build dev

2) Run with full capabilities:

       docker-compose \
         -f docker/compose/base.yml \
         -f docker/compose/gui.yml \
         -f docker/compose/gpu.yml \
         -f docker/compose/hw.yml \
         run --rm run bash

3) Install FR3 extension:

       pip install -ve extensions/rcs_fr3

4) Run example:

       cd examples
       python fr3_env_cartesian_control.py




