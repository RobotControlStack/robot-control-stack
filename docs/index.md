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
:caption: API

api/index
```

```{toctree}
:maxdepth: 2
:caption: Extensions

extensions/index
```

```{toctree}
:maxdepth: 2
:caption: Extending RCS

development/index
```

```{toctree}
:maxdepth: 2
:caption: Project Info

meta/index
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
