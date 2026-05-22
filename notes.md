
## robot setup
- switich on the power (socket button and foot button)
- create an ssh connection to transformer from multihead (the machine with the screen)
```shell
ssh -D 10000 transformer
```
- go to setttings in your browser and type proxy, set manual proxy configuration and fill socks host "localhost" and port 10000
- then open "https://192.168.101.1/" for the left, and "https://192.168.102.1/desk/" for right (our left)
- wait for the robot s to stop blinking and unlock the joints in desk

## shutdown
- press shutdown in both franka desk interfaces
- wait 2 minutes
- switch of socket and foot button
- lock the pc, dont shut it down





## policy server setup
- run the models outside the docker with the following commands; replace the path to match the checkpoint (make sure you run it in the following path: /code/duobench/robot-control-stack)
### act
```shell
uv run python -m vlagents start-server lerobot --port 20000 --host 0.0.0.0 --kwargs '{"policy_name": "act", "checkpoint_path": "/hdd_data/juelg/duobench/weights/act/transfer_cube/duobench_act_transfer_cube_real_2026-05-18_21-55-01/150000/pretrained_model/", "n_action_steps": 1}'
```

### pi05
```shell
uv run python -m vlagents start-server lerobot --port 20000 --host 0.0.0.0 --kwargs '{"policy_name": "pi05", "checkpoint_path": "/hdd_data/juelg/duobench/weights/pi05/duobench_pi05_merged_real_v1_2026-05-19_07-14-26/040000/pretrained_model", "n_action_steps": 1}'
```


### xvla
```shell
uv run python -m vlagents start-server lerobot --port 20000 --host 0.0.0.0 --kwargs '{"policy_name": "xvla", "checkpoint_path": "/hdd_data/juelg/duobench/weights/xvla/duobench_xvla_merged_real_v1_2026-05-20_23-32-13/040000/pretrained_model", "n_action_steps": 1, "rename_map": {"head": "image", "left_wrist": "image2", "right_wrist": "image3"}}'
```
## docker setup

```shell
exec su -l $USER
cd /code/duobench/robot-control-stack


# build, only once or if changes in c++
# docker build -f docker/Dockerfile -t rcs-dev .


# run docker
docker compose -f docker/compose/dev.yml run --rm rcs
bash
export PYTHONPATH="/workspace/robot-control-stack/vlagents/src"

# run the following command in the docker to start eval
python examples/inference/franka.py
```


# tasks
- for each new model and task you need to go to the [franka.json](examples/inference/franka.json) file and adapt
    - instruction: can be found in the task in [rcs_duobench/src/rcs_duobench/tasks](rcs_duobench/src/rcs_duobench/tasks)
    - record_path: "inference_recordings_<taskid>_<trainingid>_<checkpoint>"
    - for example: task=transfer_cube, modelpoath=duobench_xvla_merged_real_v1_2026-05-19_17-00-27, checkpoint=150000
- run the new policy server
- start examples/inference/franka.py or press "o" if its already running