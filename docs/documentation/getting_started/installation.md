# Installation Guide

## Requirements
RCS is developed and tested on the latest Debian and Ubuntu LTS versions.

### Step 1: Install System Dependencies
First, update your package list and install dependencies listed in `debian_deps.txt`:

- `sudo apt update`

- `sudo apt install -y $(cat debian_deps.txt)`


### Step 2: Set Up Python Virtual Environment
Create and activate a virtual environment to isolate Python dependencies:

- `python3 -m venv .venv`
- `source .venv/bin/activate`

Upgrade packaging tools and install Python dependencies:

- `pip install --upgrade pip setuptools wheel`
- `pip install -r requirements_dev.txt`

Configure pip to allow build isolation:

- `pip config --site set global.no-build-isolation false`

### Step 3: Build and Install RCS
Install the package in editable mode for active development:

- `pip install -ve .`

---


```{toctree}
:maxdepth: 1

```