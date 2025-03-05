# Installation

We build and test RCS on the latest Debian and on the latest Ubuntu LTS.

1. Install the system dependencies:
```shell
sudo apt install $(cat debian_deps.txt)
```
2. Create, activate and configure a [Python virtual environment](https://docs.python.org/3/library/venv.html):
```shell
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements_dev.txt
pip config --site set global.no-build-isolation false
```
2.5 (optional) include UTN models if you have an access token
```shell
pip config --site set install.config-settings "cmake.args=-DINCLUDE_UTN_MODELS=ON;-DGITLAB_MODELS_TOKEN=<token>"
```
3. Build and install RCS:
```shell
pip install -ve .
```