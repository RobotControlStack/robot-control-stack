#!/bin/sh

set -e
set -x

mujoco_src_dir=${1}
mujoco_version=${2}
cmake_src_dir=${3}

skip_make_sdist=false
if [ -f "${mujoco_src_dir}"/python/dist/mujoco-${mujoco_version}.tar.gz -a -d mujoco ]
then
    # Check if the local mujoco folder is up to date with the sdist generated in mujoco_src
    ln -s . mujoco-${mujoco_version}
    if tar -df "${mujoco_src_dir}"/python/dist/mujoco-${mujoco_version}.tar.gz mujoco-${mujoco_version}/mujoco
    then
        skip_make_sdist=true
    fi
    rm mujoco-${mujoco_version}
fi

if [ ${skip_make_sdist} = false ]
then
    python3 -m venv /tmp/mujoco
    bash -c 'source /tmp/mujoco/bin/activate && cd '"${1}"'/python && ./make_sdist.sh'
    tar -xf "${mujoco_src_dir}"/python/dist/mujoco-${mujoco_version}.tar.gz mujoco-${mujoco_version}
    if [ -d mujoco ]
    then
        rm -rf mujoco
    fi
    mv mujoco-${mujoco_version}/mujoco ./
    rm -rf mujoco-${mujoco_version} /tmp/mujoco
fi
find mujoco -name '*.py' -exec dirname {} + | sort | uniq | xargs realpath --relative-to=mujoco | grep -v '\.' | xargs -tI "path" mkdir -p ${cmake_src_dir}/python/mujoco/path
find mujoco -name '*.py' -exec realpath --relative-to=mujoco {} + | xargs -I "path" cp mujoco/path ${cmake_src_dir}/python/mujoco/path
