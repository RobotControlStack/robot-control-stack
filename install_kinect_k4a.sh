#!/bin/bash

# This installation script follows the instruction from (only works under debian based systems)
# https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/src/python/k4a/docs/building.md
# Dependencies for this script: git, wget, python3

# force input to continue
echo "Ensure that you have sourced a virtual python environment before continuing"
read -p "Press enter to continue"
echo "continuing..."

# store current path
cwd=$(pwd)

# create tmp folder
mkdir /tmp/kinect_k4a

# cd into tmp folder
pushd /tmp/kinect_k4a

# clone repo
git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git

# download binaries
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb

# unzip binaries, this only works under debian based systems
dpkg -x libk4a1.4_1.4.1_amd64.deb extracted


# copy files
cp -r extracted/usr/lib/x86_64-linux-gnu/libk4a.* Azure-Kinect-Sensor-SDK/src/python/k4a/src/k4a/_libs
cp -r extracted/usr/lib/x86_64-linux-gnu/libk4a1.4/libdepthengine.* Azure-Kinect-Sensor-SDK/src/python/k4a/src/k4a/_libs

pushd Azure-Kinect-Sensor-SDK/src/python/k4a

pip install build

python -m build --wheel --outdir dist/

mkdir $cwd/dist

# copy wheel file
cp dist/*.whl $cwd/dist


# ask user to install wheel package, has to input y or n
echo "Do you want to install the wheel package? (y/n)"
read install_wheel
if [ $install_wheel == "y" ]; then
    echo "Installing wheel package"
    pip install .
else
    echo "Wheel package not installed but can be found in $cwd/dist/"
fi

popd
popd