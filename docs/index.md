# Robot Control Stack

**Robot Control Stack (RCS)** is a unified and multilayered robot control interface over a MuJoCo simulation and real-world robots. It is designed to be a lean ecosystem for robot learning at scale.

```{image} _static/rcs_architecture_small.svg
:alt: RCS Architecture
:align: center
```

## Features

- **Unified Interface**: Seamlessly switch between simulation (MuJoCo) and real hardware.
- **Layered Architecture**: 
    - **High-Level**: Gymnasium-based Python API for RL and general control.
    - **Low-Level**: C++ core with Python bindings for performance-critical tasks.
- **Extensible**: Easy to add new robots and sensors via C++ or Python extensions.
- **Lean**: Minimal dependencies and overhead.

## Documentation

```{toctree}
:maxdepth: 2
:caption: Getting Started

getting_started/index
```

```{toctree}
:maxdepth: 2
:caption: User Guide

user_guide/architecture
user_guide/gym_interface
user_guide/low_level_api
```

```{toctree}
:maxdepth: 2
:caption: Extensions

extensions/overview
extensions/python_extension
extensions/cpp_extension
extensions/rcs_fr3
extensions/rcs_panda
extensions/rcs_xarm7
extensions/rcs_so101
extensions/rcs_realsense
extensions/rcs_usb_cam
extensions/rcs_tacto
extensions/rcs_robotics_library
```

```{toctree}
:maxdepth: 2
:caption: Development

contributing/index
```

## Citation

If you find RCS useful for your academic work, please consider citing it:

```bibtex
@misc{juelg2025robotcontrolstack,
  title={{Robot Control Stack}: {A} Lean Ecosystem for Robot Learning at Scale}, 
  author={Tobias J{\"u}lg and Pierre Krack and Seongjin Bien and Yannik Blei and Khaled Gamal and Ken Nakahara and Johannes Hechtl and Roberto Calandra and Wolfram Burgard and Florian Walter},
  year={2025},
  howpublished = {\url{https://arxiv.org/abs/2509.14932}}
}
```

For more scientific info, visit the [paper website](https://robotcontrolstack.github.io/).
