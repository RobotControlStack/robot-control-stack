# Robot Control Stack

**Robot Control Stack (RCS)** is a unified and multilayered robot control interface over a MuJoCo simulation and real world robot currently implemented for the FR3/Panda, xArm7, UR5e and SO101.

## Installation

We build and test RCS on the latest Debian and on the latest Ubuntu LTS.

1.  **System Dependencies**:
    ```shell
    sudo apt install $(cat debian_deps.txt)
    ```

2.  **Python Environment**:
    ```shell
    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements_dev.txt
    ```

3.  **Install RCS**:
    ```shell
    pip install -ve .
    ```

## Hardware Extensions

RCS supports various hardware extensions (e.g., FR3, xArm7, RealSense). These are located in the `extensions` directory.

To install an extension:

```shell
pip install -ve extensions/rcs_fr3
```

For a full list of extensions and detailed documentation, visit [robot-control-stack.org/extensions](https://robot-control-stack.org/extensions).

## Documentation


For full documentation, including installation, usage, and API reference, please visit:

**[robot-control-stack.org](https://robot-control-stack.org)**

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