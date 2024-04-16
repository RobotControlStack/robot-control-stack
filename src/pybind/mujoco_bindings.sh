#!/bin/sh

python3 -m venv /tmp/mujoco
cd ${1}/python
bash -c "source /tmp/mujoco/bin/activate && ./make_sdist.sh"
tar -xf dist/mujoco-*.tar.gz
