# check cam

```shell
# activate venv
source .venv311/bin/activate

# check the camera id and stream
python -m rcs_realsense
python RCSToolBox/src/rcs_toolbox/camera_liveviewer.py

# run vla
sudo chmod 777 /dev/ttyUSB0
python RCSToolBox/src/rcs_toolbox/run_vla.py

# rcs fr3 control
python -m rcs_fr3 --help
python -m rcs_fr3 home 192.168.103.1
python -m rcs_fr3 home -u --no-franka-hand 192.168.103.1
python -m rcs_fr3 lock 192.168.103.1
python -m rcs_fr3 unlock 192.168.103.1

## mujuco viewer
python -m mujoco.viewer
```


python RCSToolBox/src/rcs_t