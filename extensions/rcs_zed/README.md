# Zed Camera Extension

## Installation
- You need a PC with an Nvidia GPU and CUDA 12.8 installed (12.x is probably fine)
- Download and install the Zed SDK from [here](https://www.stereolabs.com/en-fr/developers/release)
- Install the python bindings with the manual [here](https://github.com/stereolabs/zed-python-api)

Or just use the docker shipped with RCS. Checkout [this readme](../../docker/README.md) to build and start the docker. The tools shown below are available inside the docer container.

Package installation:
```shell
pip install -ve .
```


## Usage


```shell
python -m rcs_zed serials
python -m rcs_zed rgb-view
```

### Permissions
```shell
# if in docker, run this on your host system
sudo usermod -a -G video,plugdev $USER
```

### Tools to check your Zed
Most relevant is the `ZED_Diagnostic`, use this to check what exactly is not working.

| Command / Tool | What it does | When to use it |
| :--- | :--- | :--- |
| **`ZED_Explorer`** | Live video feed & recording | To check image quality, adjust exposure, and record **.SVO** files. |
| **`ZED_Depth_Viewer`** | 3D Depth & Point Cloud | To see the "Neural" depth in action and check 3D accuracy. |
| **`ZED_Diagnostic`** | Hardware/Software Health | Use this first if something feels "broken." |
| **`ZED_Sensor_Viewer`** | IMU & Magnetometer | To see real-time data from the accelerometer and gyroscope. |
| **`ZED_Studio`** | Multi-Camera Management | If you have more than one ZED connected at once. |
| **`ZEDfu`** | Spatial Mapping | To "scan" a room and create a 3D mesh in real-time. |

### Issues with the connection
- see [this article](https://support.stereolabs.com/hc/en-us/articles/207635225-How-to-fix-USB-3-0-bandwidth-and-connection-issues)