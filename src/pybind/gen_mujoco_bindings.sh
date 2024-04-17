#!/bin/sh

set -e
set -x

mujoco_src_dir=${1}
mujoco_version=${2}

python3 -m venv /tmp/mujoco
bash -c 'source /tmp/mujoco/bin/activate && cd '"${1}"'/python && ./make_sdist.sh'
tar -xf "${mujoco_src_dir}"/python/dist/mujoco-${mujoco_version}.tar.gz mujoco-${mujoco_version}
rm -rf mujoco
mv mujoco-${mujoco_version}/mujoco ./
rm -rf mujoco-${mujoco_version}
